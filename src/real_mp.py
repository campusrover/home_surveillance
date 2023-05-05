#!/usr/bin/env python

from random import randint
from threading import Thread
from threading import Lock
from threading import Event
import time
import copy

import rospy
import rosnode
from std_msgs.msg import String

from real_patrol import Patrol
from real_patrol import Zones

class MP:
    def __init__(self, name):
        self.name = name
        self.state = 'follower'
        self.term = 0
        self.voted_for = ''
        self.cur_leader = ''
        self.election_timeout = self.__get_rand_duration(2000, 3000)
        self.paid_homage_at = rospy.Time.now()
        self.mp_count = 4
        self.majority = self.mp_count // 2 + 1
        self.other_mps = {'roba', 'robb', 'robc', 'rafael'}
        self.other_mps.remove(self.name)
        self.homage_request_thread = None
        self.resignation_thread = None
        self.follower_death_handler_thread = None
        self.dynamic_patrol_threads = set()
        self.should_resign = False
        self.guard_map = None
        self.zone_map = None
        self.global_interrupt_map = {"resignation": Event(), "robbery": Event(), "leader_shutdown": Event()}
        self.local_interrupt_map = {"roba_shutdown": Event(), "robb_shutdown": Event(), "robc_shutdown": Event(), "rafael_shutdown": Event()}
        
        #=====For robbery event simulations====================
        rospy.Subscriber('keys', String, self.__key_input_handler)
        #============================================================

        #=====For LEA testing=====
        self.election_result_publisher = rospy.Publisher(f'{self.name}_election_result_broadcasts', String, queue_size=10)
        #=========================

        self.vote_request_publisher = rospy.Publisher(f'{self.name}_vote_requests', String, queue_size=10)
        self.homage_request_publisher = rospy.Publisher(f'{self.name}_homage_requests', String, queue_size=10)

        self.vote_reply_publisher_to = {}
        for mp_name in self.other_mps:
            rospy.Subscriber(f'{mp_name}_vote_requests', String, self.__vote_request_handler)
            self.vote_reply_publisher_to[mp_name] = rospy.Publisher(f'{self.name}_vote_replies_to_{mp_name}', String, queue_size=10)
            rospy.Subscriber(f'{mp_name}_homage_requests', String, self.__homage_request_handler)

    def attend_session(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.state == 'follower':
                self.__follow()
            elif self.state == 'candidate':
                self.__start_election()
            elif self.state == 'leader':
                self.__lead()
            rate.sleep()

    def kill_action_servers(self):
        if self.state == 'leader':
            self.global_interrupt_map["leader_shutdown"].set()

    #=======Session Methods=======

    def __follow(self):
        if self.__discharged():
            self.__remove_dead_mps()
            self.term += 1
            self.state = 'candidate'

    def __start_election(self):
        ballot = Ballot()
        ballot.yea = 1
        
        subs = []
        for mp_name in self.other_mps:
            subs.append(rospy.Subscriber(f'{mp_name}_vote_replies_to_{self.name}', String, self.__vote_reply_handler, ballot))

        time.sleep(1)

        self.vote_request_publisher.publish(f'{self.term}, {self.name}')
        
        self.__wait_for_replies(ballot)
        
        if ballot.yea >= self.majority and self.state != 'follower':
            self.state = 'leader'
        else:
            self.__become_follower()

        for sub in subs:
            sub.unregister()

    def __lead(self):
        self.cur_leader = self.name
        if self.homage_request_thread is None:
            self.homage_request_thread = Thread(target=self.__send_homage_requests, daemon=True)
            self.homage_request_thread.start()
            self.resignation_thread = Thread(target=self.__resignation_handler, daemon=True)
            self.resignation_thread.start()
            self.follower_death_handler_thread = Thread(target=self.__follower_death_handler, daemon=True)
            self.follower_death_handler_thread.start()
        elif self.should_resign:
            self.homage_request_thread.join()
            self.homage_request_thread = None
            self.resignation_thread.join()
            self.global_interrupt_map["resignation"].clear()
            self.resignation_thread = None
            self.follower_death_handler_thread.join()
            self.follower_death_handler_thread = None
            self.dynamic_patrol_threads = set()
            self.__become_follower()
        else:
            patrols = set()
            mps = self.other_mps.copy()
            mps.add(self.name)
            patrols.add(Patrol(self.global_interrupt_map, self.local_interrupt_map, self.name, Zones.zone_map["A"]))
            self.guard_map = {}
            self.zone_map = {}

            for guard, zone_name in zip(self.other_mps, Zones.secondary_zones):
                patrols.add(Patrol(self.global_interrupt_map, self.local_interrupt_map, guard, Zones.zone_map[zone_name]))
                self.guard_map[zone_name] = guard
                self.zone_map[guard] = zone_name

            self.election_result_publisher.publish(f"leader: {self.name} // {self.guard_map}")

            patrol_threads = set()
            for patrol in patrols:
                patrol_threads.add(Thread(target=patrol.execute, daemon = True))

            for thread in patrol_threads:
                thread.start()
  
            for thread in patrol_threads:
                thread.join()

    #=======Callback Methods=======

    def __resignation_handler(self):
        while True:
            if self.global_interrupt_map["resignation"].is_set():
                self.global_interrupt_map["resignation"].set()
                self.should_resign = True
                break

    def __vote_request_handler(self, msg):
        received_term, candidate = self.__parse_message(msg)
        self.__update_term(received_term)
        if received_term >= self.term and (self.voted_for=='' or self.voted_for==candidate):
            self.voted_for = candidate
            self.vote_reply_publisher_to[candidate].publish(f'{self.term}, yea')
        else:
            self.vote_reply_publisher_to[candidate].publish(f'{self.term}, nay')

    def __vote_reply_handler(self, msg, ballot):
        self.__mark_ballot(msg, ballot)

    def __homage_request_handler(self, msg):
        received_term, pretender = self.__parse_message(msg)
        self.__update_term(received_term)
        if received_term >= self.term:
            self.cur_leader = pretender
            self.paid_homage_at = rospy.Time.now()
            self.state = 'follower'
            self.voted_for = ''

    def __follower_death_handler(self):
        while not self.global_interrupt_map["resignation"].is_set():
            obituaries = self.__remove_dead_mps()
            for dead_follower in obituaries:
                self.local_interrupt_map[f"{dead_follower}_shutdown"].set()
                vacant_zone = self.zone_map[dead_follower]
                self.__del_from_guard_zone_maps(dead_follower, vacant_zone)
                vacant_zone_priority = Zones.secondary_zones_priority_map[vacant_zone]
                if vacant_zone_priority != Zones.lowest_priority and self.guard_map.get(Zones.secondary_zones[vacant_zone_priority + 1]) is not None:
                    for zone in reversed(Zones.secondary_zones):
                        new_guard = self.guard_map.get(zone)
                        if new_guard is not None:
                            self.__del_from_guard_zone_maps(new_guard, self.zone_map[new_guard])
                            self.local_interrupt_map[f"{new_guard}_shutdown"].set()
                            self.local_interrupt_map = self.local_interrupt_map.copy()
                            self.local_interrupt_map[f"{new_guard}_shutdown"] = Event()
                            new_patrol = Patrol(self.global_interrupt_map, self.local_interrupt_map, new_guard, Zones.zone_map[vacant_zone])
                            self.zone_map[new_guard] = vacant_zone
                            self.guard_map[vacant_zone] = new_guard
                            new_patrol_thread = Thread(target=new_patrol.execute, daemon=True)
                            self.dynamic_patrol_threads.add(new_patrol_thread)
                            new_patrol_thread.start()
                            break
        for thread in self.dynamic_patrol_threads:
            thread.join()

    #=====For resignation/re-election testing====================
    def __key_input_handler(self, msg):
        if msg.data[0] == 'r':
            self.global_interrupt_map["resignation"].set()
        if msg.data[0] == 'd':
            print('hello')
            self.global_interrupt_map["robbery"].set()
    #============================================================

    #=======Helper Methods=======
    def __del_from_guard_zone_maps(self, guard_name, zone_name):
        del self.zone_map[guard_name]
        del self.guard_map[zone_name]

    def __send_homage_requests(self):
        while not self.global_interrupt_map["resignation"].is_set():
            self.homage_request_publisher.publish(f'{self.term}, {self.name}')

    def __get_rand_duration(self, start_msec, end_msec):
        rand_sec = randint(start_msec, end_msec) * 0.001
        return rospy.Duration(rand_sec)

    def __discharged(self):
        return rospy.Time.now() - self.paid_homage_at > self.election_timeout

    def __update_term(self, received_term):
        if received_term > self.term:
            self.term = received_term
            self.voted_for = ''

    def __wait_for_replies(self, ballot):
        while True:
            vote_count = 0
            ballot.mutex.acquire()
            try:
                vote_count = ballot.yea + ballot.nay
            finally:
                ballot.mutex.release()
            if vote_count == self.mp_count:
                break
    
    def __parse_message(self, msg):
        split_msg = msg.data.split(', ')
        return int(split_msg[0]), split_msg[1]

    def __mark_ballot(self, msg, ballot):
        received_term, vote = self.__parse_message(msg)
        self.__update_term(received_term)
        ballot.mutex.acquire()
        try: 
            if vote == 'yea':
                ballot.yea += 1
            else:
                ballot.nay += 1
        finally:
            ballot.mutex.release()

    def __become_follower(self):
        self.state = 'follower'
        self.voted_for = ''
        self.paid_homage_at = rospy.Time.now()
        self.election_timeout = self.__get_rand_duration(2000, 3000)
        self.should_resign = False
    
    def __remove_dead_mps(self):
        obituary = set()
        mp_map = {}
        for mp in self.other_mps:
            mp_map[f"/mp_{mp}"] = mp

        live_nodes = rosnode.get_node_names()
        for mp_node in mp_map:
            if mp_node not in live_nodes:
                dead_mp = mp_map[mp_node]
                obituary.add(dead_mp)
                self.other_mps.remove(dead_mp)
                self.majority -= 1
                self.mp_count -= 1
        return obituary

class Ballot:
    def __init__(self):
        self.yea = 0
        self.nay = 0
        self.mutex = Lock()

if __name__ == '__main__':
    rospy.init_node('mp')
    robot_name = rospy.get_param('~robot_name')
    mp = MP(robot_name)
    rospy.on_shutdown(mp.kill_action_servers)
    mp.attend_session()

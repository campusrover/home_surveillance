#!/usr/bin/env python

from random import randint
from std_msgs.msg import String
from threading import Lock
import time

import rospy

class MP:
    def __init__(self, name):
        self.name = name
        self.state = 'follower'
        self.term = 0
        self.voted_for = ''
        self.election_timeout = self.__get_rand_duration(2000, 3000)
        self.paid_homage_at = rospy.Time.now()
        self.mp_count = 4
        self.majority = self.mp_count // 2 + 1
        self.other_mps = {'roba', 'robb', 'robc', 'rafael'}
        self.other_mps.remove(self.name)

        self.hegemony_publisher = rospy.Publisher(f'{self.name}_hegemony_broadcasts', String, queue_size=10)

        self.vote_request_publisher = rospy.Publisher(f'{self.name}_vote_requests', String, queue_size=10)
        self.homage_request_publisher = rospy.Publisher(f'{self.name}_homage_requests', String, queue_size=10)

        self.vote_reply_publisher_to = {}
        self.homage_reply_publisher_to = {}
        for mp_name in self.other_mps:
            rospy.Subscriber(f'{mp_name}_vote_requests', String, self.__vote_request_handler)
            self.vote_reply_publisher_to[mp_name] = rospy.Publisher(f'{self.name}_vote_replies_to_{mp_name}', String, queue_size=10)
            rospy.Subscriber(f'{mp_name}_homage_requests', String, self.__homage_request_handler)
            # self.homage_reply_publisher_to[mp_name] = rospy.Publisher(f'{self.name}_homage_replies_to_{mp_name}', String, queue_size=10)

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

    #=======Session Methods=======

    def __follow(self):
        if self.__discharged():
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

        if ballot.yea >= self.majority:
            self.state = 'leader'
        else:
            self.state = 'follower'
            self.paid_homage_at = rospy.Time.now()
            self.election_timeout = self.__get_rand_duration(2000, 3000)

        for sub in subs:
            sub.unregister()

    def __lead(self):
        self.homage_request_publisher.publish(f'{self.term}, {self.name}')
        self.hegemony_publisher.publish(self.name)
        # ballot = Ballot()
        # ballot.yea = 1

        # subs = []
        # for mp_name in self.other_mps:
        #     subs.append(rospy.Subscriber(f'{mp_name}_homage_replies_to_{self.name}', String, self.__homage_reply_handler, ballot))

        # time.sleep(1)

        # self.homage_request_publisher.publish(f'{self.term}, {self.name}')

        # self.__wait_for_replies(ballot)

        # if ballot.yea >= self.majority:
        #     self.state = 'leader'
        #     self.hegemony_publisher.publish(self.name)
        # else:
        #     self.state = 'follower'
        #     self.paid_homage_at = rospy.Time.now()
        #     self.election_timeout = self.__get_rand_duration(2000, 3000)

        # for sub in subs:
        #     sub.unregister()


    #=======Callback Methods=======

    def __vote_request_handler(self, msg):
        received_term, candidate = self.__parse_message(msg)
        self.__update_term(received_term)
        if received_term >= self.term and (self.voted_for=='' or self.voted_for==candidate):
            self.vote_reply_publisher_to[candidate].publish(f'{self.term}, yea')
            # print(f"{self.name} is voting yea for {candidate}")
        else:
            self.vote_reply_publisher_to[candidate].publish(f'{self.term}, nay')
            # print(f"{self.name} is voting nay for {candidate}")

    def __vote_reply_handler(self, msg, ballot):
        # print(f"{self.name}, f{msg}")
        self.__mark_ballot(msg, ballot)

    def __homage_request_handler(self, msg):
        received_term, pretender = self.__parse_message(msg)
        self.__update_term(received_term)
        if received_term >= self.term:
            self.paid_homage_at = rospy.Time.now()
            self.state = 'follower'
            self.voted_for = ''
            # self.homage_reply_publisher_to[pretender].publish(f'{self.term}, yea')
        # else:
            # self.homage_reply_publisher_to[pretender].publish(f'{self.term}, nay')

    # def __homage_reply_handler(self, msg, ballot):
    #     self.__mark_ballot(msg, ballot)
        

    #=======Helper Methods=======

    def __get_rand_duration(self, start_msec, end_msec):
        rand_sec = randint(start_msec, end_msec) * 0.001
        return rospy.Duration(rand_sec)

    def __discharged(self):
        return rospy.Time.now() - self.paid_homage_at > self.election_timeout

    def __update_term(self, received_term):
        if received_term > self.term:
            self.term = received_term
            if self.state != 'candidate':
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
        # print(f"{self.name}'s ballot: yea={ballot.yea} nay={ballot.nay}")

class Ballot:
    def __init__(self):
        self.yea = 0
        self.nay = 0
        self.mutex = Lock()


if __name__ == '__main__':
    rospy.init_node('mp')
    robot_name = rospy.get_param('~robot_name')
    mp = MP(robot_name)
    mp.attend_session()

Think of an interesting way to use the result of leader election. One thing that's distinctive about the LEA is that it constantly sends a message over the `homage_request_publisher` that establishes its authority. So it's logical that the interesting aspect be coded in the `__homage_request_handler`, and that the messages that the `homage_request_publisher` carries have substantial content (e.g., encodings of what the leader expects the followers to do).

Another place to implement interesting actions would be in the `__lead` function itself, where the leader might have additional publications on topics besides the `homage_request_publisher`. 

Another way that's interesting would be to use the `bondpy` package to have a node check when another is shutdown, and use that together with the method of catching a `roskill`, to implement basic failure recovery. E.g., when the leader node dies, the LEA adjusts and elects a new leader.

cf. the two links below:
https://answers.ros.org/question/240671/catching-a-rosnode-kill-a/
http://wiki.ros.org/bondpy
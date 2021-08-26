#!/usr/bin/env python
import platform
if platform.release() == '4.9.140-tegra':
    import sys
    sys.path.append('/usr/lib/python2.7/dist-packages') # in order to import cv2 under python3
    import rospy
    sys.path.remove('/usr/lib/python2.7/dist-packages') # append back in order to import rospy
else:
    import rospy
import asyncio
from dataclasses import dataclass
from io import BytesIO
from threading import Thread
from dataclasses import dataclass
from typing import Any


@dataclass
class Topic:
    name: str
    type: Any

class RosHandler():
    def __init__(self, our_peer_id, peer_id, channel, event_loop, topics_to_send=[]):
        self.our_peer_id = our_peer_id
        self.peer_id = peer_id
        self.topics_to_send = topics_to_send
        self.channel = channel
        self.event_loop = asyncio.get_running_loop() 

        self.publishers = {}
        rospy.on_shutdown(self._shutdown_callback)

    def start(self):
        """Spins up ROS background thread; must be called to start
        receiving data

        :return: itself
        :rtype: LocalizationInterface
        """
        Thread(target=self._init_and_spin_ros, args=()).start()
        return self

    def _shutdown_callback(self):
        pass

    def _init_and_spin_ros(self):
        rospy.loginfo("Starting WebRTC Node for "
                      + str(self.our_peer_id) + " with " + str(self.peer_id))
        self._start_listen()
        self._start_publish()

        # rospy.spin()

    def _message_serializor(self, msg):
        buff = BytesIO()
        msg.serialize(buff)
        return buff.getvalue()

    def _general_callback(self, msg):
        serialized_msg = self._message_serializor(msg)
        # print(serialized_msg)
        # self.event_handler.emit("send", serialized_msg)
        # print(self.channel.readyState)
        if(self.channel.readyState == "open"):
            print("serialized", serialized_msg)
            # self.channel.send("hej")
            loop = self._get_or_create_eventloop()
            loop.run_until_complete(self.channel.send("hej"))

            # asyncio.run(self.channel.send("hej"))
           # self.channel.send("hej")
            # asyncio.get_running_loop().create_task(self.channel.send(serialized_msg))

    def _get_or_create_eventloop(self):
        try:
            return asyncio.get_event_loop()
        except RuntimeError as ex:
            if "There is no current event loop in thread" in str(ex):
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                return asyncio.get_event_loop()

    def _start_listen(self):
        for topic in self.topics_to_send:
            rospy.Subscriber(f"{self.our_peer_id}/{topic.name}",
                             topic.type,
                             self._general_callback,
                             queue_size=10)

        # rospy.Subscriber(self._state_topic,
                         # type(self.state.state_msg),
                         # self._read_state_msg,
                         # tcp_nodelay=True,
                         # queue_size=1)

    def _start_publish(self):
        pass

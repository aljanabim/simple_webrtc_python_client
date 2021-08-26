import platform
if platform.release() == '4.9.140-tegra':
    import sys
    sys.path.append('/usr/lib/python2.7/dist-packages') # in order to import cv2 under python3
    import rospy
    sys.path.remove('/usr/lib/python2.7/dist-packages') # append back in order to import rospy
else:
    import rospy

import asyncio
import queue
from svea_msgs.msg import lli_ctrl
from std_msgs.msg import String 
from .ros_handler import RosHandler, Topic
from io import BytesIO

q = queue.Queue(maxsize=10)

async def start(channel):
    while True:
        if not q.empty():
            msg = q.get()
            if channel and channel.readyState == "open":
                channel.send(msg)
        await asyncio.sleep(1e-5)

def message_serializor(msg):
        buff = BytesIO()
        msg.serialize(buff)
        return buff.getvalue()

def cb(msg):
    serialized_msg = message_serializor(msg)
    q.put(serialized_msg)

def data_channel_handler(our_peer_id, our_peer_type, peers, peer_id):
    """
    Handler of the RTCDataChannel. In its three event listeners (on_open, on_message, on_close) you specify what to do with the data sent across the data channel over the peer connection.

    Args:
        our_peer_id (string): Peer ID of this peer
        our_peer_type (string): Peer Type of this peer
        peers (dict): Dictionary of all connected peers
        peer_id (string): ID of the peer
    """
    peer = peers.get(peer_id)
    channel = peer.get('dataChannel')
    topics_to_send = [Topic("chatter", String)]
    # ee = EventEmitter()
    # @ee.on("send")
    # def on_send(msg):
        # if(channel.readyState == "open"):
            # channel.send(msg)

    rospy.Subscriber('/svea1/lli/ctrl_request', lli_ctrl, cb)
    # event_loop = asyncio.get_running_loop()

    # ros_handler = RosHandler(our_peer_id, peer_id, channel, asyncio.get_running_loop(), topics_to_send)

    @channel.on("open")
    def on_open():
        print("Data channel with", peer_id, "is open")
        channel.send("Hello from {}".format(our_peer_id))

    @channel.on("close")
    def on_close():
        print("Data channel with", peer_id, "has been closed")

    @channel.on("message")
    def on_message(message):
        print(peer_id, "says", message)

    # ros_handler._init_and_spin_ros()
    # rospy.Subscriber('/Right/chatter', String, lambda msg: cb(channel, ee, msg))
    asyncio.Task(start(channel))


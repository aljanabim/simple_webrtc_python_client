import platform
if platform.release() == '4.9.140-tegra':
    import sys
    sys.path.append('/usr/lib/python2.7/dist-packages') # in order to import cv2 under python3
    import rospy
    sys.path.remove('/usr/lib/python2.7/dist-packages') # append back in order to import rospy
else:
    import rospy
import time
import asyncio
import queue
from svea_msgs.msg import lli_ctrl
from std_msgs.msg import String 
from .ros_handler import RosHandler, Topic
from io import BytesIO, StringIO

q = queue.Queue(maxsize=10)

async def start(channel):
    while True:
        if not q.empty():
            msg = q.get()
            if channel and channel.readyState == "open":
                channel.send(msg)
        await asyncio.sleep(1e-4) # 1e-5 causes CPU to take 100% usage of one of the cores

def message_serializor(msg):
        buff = BytesIO()
        msg.serialize(buff)
        return buff.getvalue()

def cb(msg):
    serialized_msg = message_serializor(msg)
    # print(serialized_msg)
    q.put_nowait(serialized_msg)

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
    # topics_to_send = [Topic("chatter", String)]

    # rospy.Subscriber('/svea1/lli/ctrl_request', lli_ctrl, cb)
    # pub = rospy.Publisher(f"/{our_peer_id}/lli/ctrl_request", lli_ctrl, queue_size=1)
    ping = rospy.Publisher("ping", String)
    def pong_cb(msg):
        m = msg.data.split(" ")
        start_time = float(m[-1])
        idx = int(m[1])
        print("Pong", idx, time.time()-start_time)
    rospy.Subscriber('pong', String, pong_cb)

    # event_loop = asyncio.get_running_loop()

    # ros_handler = RosHandler(our_peer_id, peer_id, channel, asyncio.get_running_loop(), topics_to_send)

    @channel.on("open")
    def on_open():
        print("Data channel with", peer_id, "is open")
        # channel.send("Hello from {}".format(our_peer_id))
        for i in range(100):
            ping.publish("Ping" + " " + str(i) + " " + str(time.time()))
            # channel.send("Ping" + " " + str(i) + " " + str(time.time()))


    @channel.on("close")
    def on_close():
        print("Data channel with", peer_id, "has been closed")

    @channel.on("message")
    def on_message(message):
        m = message.split(" ")
        start_time = float(m[-1])
        idx = int(m[1])
        print("Pong", idx, time.time()-start_time)
        if idx == 99:
            input("Ping?")
            for i in range(100):
                channel.send("Ping" + " " + str(i) + " " + str(time.time()))
        # channel.send("Ping " + str(time.time()))
        # print(message, message[0:4])
        # pub.publish(lli_ctrl().deserialize(message))
        # print(lli_ctrl().deserialize(message))
        # print(peer_id, "says", message)

    # ros_handler._init_and_spin_ros()
    # rospy.Subscriber('/Right/chatter', String, lambda msg: cb(channel, ee, msg))
    # asyncio.Task(start(channel))


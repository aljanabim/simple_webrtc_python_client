#!/usr/bin/env python
# IMPORTS
from dotenv import load_dotenv
from os import environ
import argparse
import asyncio
import socket
from lib.signaling_channel import SignalingChannel
from lib.webrtc_manager import WebrtcManager
from lib.webrtc_handlers.data_channel_handler import data_channel_handler
import rospy
# PARSE ARGUMENTS
hostname = socket.gethostname()
parser = argparse.ArgumentParser()
parser.add_argument("--id", required=False,
                    default=''.join(e for e in hostname if e.isalnum()))  # use id if provided, else use hostname and strip it from special characters
parser.add_argument("--dev", required=False,
                    action=argparse.BooleanOptionalAction, default=False)
args = parser.parse_args()

# LOAD ENVIRONMENT VARIABLES
if(args.dev):
    load_dotenv('../../config/dev.env')
else:
    load_dotenv('../../config/prod.env')

# CONSTANTS
TOKEN = environ.get('TOKEN')
SIGNALING_SERVER_URL = environ.get('SIGNALING_SERVER_URL')
PEER_ID = args.id
PEER_TYPE = "admin"

# SETUP SIGNALING CHANNEL AND WEBRTC
channel = SignalingChannel(PEER_ID, PEER_TYPE, SIGNALING_SERVER_URL, TOKEN)
webrtcOptions = {"enableDataChannel": True,
                 "enableLocalStream": True,
                 "enableRemoteStream": False,
                 "dataChannelHandler": data_channel_handler}

if PEER_ID == "Right":
    webrtcOptions = {"enableDataChannel": True,
                     "enableLocalStream": False,
                     "enableRemoteStream": True,
                     "dataChannelHandler": data_channel_handler}

manager = WebrtcManager(PEER_ID, PEER_TYPE, channel,
                        webrtcOptions, verbose=True)

if __name__ == "__main__":
    rospy.init_node(PEER_ID+"_webrtc")
    asyncio.run(
        channel.connect(),
        debug = True
    )

'''
    YOUR CODE HERE - In this file you can right the overall logic of your application
    once a succesfull peer - to - peer connetion is established.
'''

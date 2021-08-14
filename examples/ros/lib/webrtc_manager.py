import asyncio
from aiortc import RTCIceCandidate, RTCPeerConnection, RTCSessionDescription, RTCConfiguration, RTCIceServer, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer, MediaRecorder, MediaRelay
import platform
from .media_handlers import StreamViewer
# import cv2

class WebrtcManager():
    """
    Manages the connections and signaling logic between all peers in a meshed network. Then negotiation logic, used during the signaling process, is based on "The perfect negotiation logic" Reference: https://,,,

    Args:
        our_peer_id (string): ID assigned to this peer
        our_peer_type (string): What type of peer this is. It is up to the application. For example our_peer_type can be "admin", "vehicle", "controltower" or "robot" depeding on you application
        signaling_channel (SignalingChannel): An instance of the signaling channel
        options (dict): Options of the WebRTC connection. Contains ["enableDataChannel (bool)","enableStreams (bool)","dataChannelHandler (func)"]
        verbose (bool, optional): If true the manager will print its status when events occur. Defaults to False.
    """

    def __init__(self, our_peer_id, our_peer_type, signaling_channel, options, verbose=False):
        self.our_peer_id = our_peer_id
        self.our_peer_type = our_peer_type
        self.signaling_channel = signaling_channel
        self.signaling_channel.socket.on(
            "message", self.signaling_message_handler)
        self.signaling_channel.socket.on(
            "disconnect", self.signaling_disconnect_handler)
        self.peers = {}
        self.options = options
        self.verbose = verbose
        self.iceServers = [
            {"urls": "stun:stun.l.google.com:19302"},
            {"urls": "stun:stun1.l.google.com:19302"},
            {"urls": "stun:stun2.l.google.com:19302"}]

    async def signaling_message_handler(self, message):
        """
        Replaces the on_message handler of the signaling channel. Takes care of the signaling message exchange

        Args:
            message (dict): The message from the signaling channel
        """
        # print('Message give to manager', message)
        _from = message.get("from")
        payload = message.get("payload")
        action, connections, be_polite, sdp, ice = payload.get("action"), payload.get(
            "connections"), payload.get("bePolite"), payload.get("sdp"), payload.get("ice")

        if(action == "open"):
            for new_peer in connections:
                await self.add_peer(new_peer.get("peerId"),
                                    new_peer.get("peerType"), be_polite)

        elif(action == "close"):
            await self.remove_peer(self.peers[_from])

        elif(action == "sdp"):
            if(self.verbose):
                print("Recived {} from {}".format(sdp.get('type'), _from))
            await self.update_session_description(self.peers[_from], sdp)

        elif(action == "ice"):
            await self.update_ice_candidate(self.peers[_from], ice)
        else:
            if(self.verbose):
                print("Received an unkown action {}".format(action))

    async def signaling_disconnect_handler(self):
        """
        Replaces the on_disconnect handler of the signaling channel. Takes care of temove the connections of all registered peers.
        """
        # make a copy of all regisered peer ids since remove_peers deletes the peer and changes the dict to change which ruins the for loop
        peer_ids = list(self.peers.keys())
        for peer_id in peer_ids:
            await self.remove_peer(self.peers[peer_id])

    async def add_peer(self, peer_id, peer_type, polite):
        """

        Takes care of adding a peer to the connection and all the corresponding signaling logic

        Args:
            peer_id (string): ID of the peer to be added
            peer_type (string): Type of peer to be added
            polite (bool): Wether to be a plite peer during the signaling process
        """
        if(peer_id in self.peers):
            if(self.verbose):
                print("A peer connection with id", peer_id, "already exists")
        else:
            config = RTCConfiguration(
                [RTCIceServer(server.get("urls")) for server in self.iceServers])
            self.peers[peer_id] = {
                "peerId": peer_id,
                "peerType": peer_type,
                "polite": polite,
                "rtcPeerConnection": RTCPeerConnection(config),
                "dataChannel": None,
                "stream": None,
                "remoteStream": None,
                "makingOffer": False,
                "ignoreOffer": False,
                "isSettingRemoteAnswerPending": False,
            }

            if(self.options.get("enableDataChannel")):
                self.peers[peer_id]["dataChannel"] = self.peers[peer_id]['rtcPeerConnection'].createDataChannel(
                    f"{peer_id}Channel",
                    # the application assumes that data channels are created manually on both peers
                    negotiated=True,
                    # data channels created with the same id are connected to each other across peers
                    id=0
                )

                # TODO
                # try:
                self.options["dataChannelHandler"](
                    self.our_peer_id, self.our_peer_id, self.peers, peer_id)
                # except:
                    # print(
                        # "Something with the data_channel_handler initialization went wrong")
            # if(self.options.get("enableLocalStream")):
                # self.update_local_stream(self.peers[peer_id])
            if self.options.get('enableRemoteStream'):
                peer = self.peers[peer_id]
                peer['remoteStream'] = StreamViewer(peer_id+'.mp4')
                peerConnection = peer.get('rtcPeerConnection')
                peerConnection.addTransceiver('video', direction = 'sendrecv');
                peerConnection.addTransceiver('audio', direction = 'sendrecv');
                @peerConnection.on('track')
                def on_track(track):
                    peer['remoteStream'].addTrack(track)
                    print('### Got this track', track)


            await self.update_negotiation_logic(self.peers[peer_id])

    async def update_negotiation_logic(self, peer):
        """
        Updates the negotiation logic of a new peer. Makes the RTCPeerConnection trigger and offer generation if the peers is impolite.

        Args:
            peer (dict): Data about the peer. Containes ["peerId"(string), "peerId" (string),"peerType" (string),"polite" (bool),"rtcPeerConnection" (RTCPeerConnection),"dataChannel" (RTCDataChannel),"makingOffer" (bool),"ignoreOffer" (bool),"isSettingRemoteAnswerPending" (bool)]
        """
        peerConnection = peer.get('rtcPeerConnection')

        # @peerConnection.on("onicecandidate") # is not necessary since aiortc sends the since aiortc gathers all candidates in one go, so you already have all the candidates in the offer / answer it generates. https://github.com/aiortc/aiortc/issues/246

        try:
            if(not peer["polite"]):
                peer["makingOffer"] = True
                if(self.options.get('enableLocalStream')):
                    self.update_local_streams(peer)
                offer = await peerConnection.createOffer()
                await peerConnection.setLocalDescription(offer)
                if(self.verbose):
                    print("Sending offer to {}".format(peer["peerId"]))
                await self.signaling_channel.send_to(
                    peer['peerId'], {
                        "action": "sdp",
                        "sdp": {
                            "sdp": peerConnection.localDescription.sdp,
                            "type": peerConnection.localDescription.type}})
        except:
            print('Something related to update_negotiation_logic went wrong')

        finally:
            peer["makingOffer"] = False

    async def update_session_description(self, peer, description):
        """
        The logic to update the session description protocol (SDP) during negotiations

        Args:
            peer (dict): Data about the peer. Containes ["peerId"(string), "peerId" (string),"peerType" (string),"polite" (bool),"rtcPeerConnection" (RTCPeerConnection),"dataChannel" (RTCDataChannel),"makingOffer" (bool),"ignoreOffer" (bool),"isSettingRemoteAnswerPending" (bool)]
            description (dict): Contains ["spd", "type"]
        """
        try:
            rtc_decription = RTCSessionDescription(
                description["sdp"], description["type"])
            peerConnection = peer['rtcPeerConnection']
            # if we recived and offer, check if there is an offer collision(ie. we already have created a local offer and tried to send it)
            offerCollision = description["type"] == "offer" and (
                peer["makingOffer"] or peerConnection.signalingState != "stable")
            peer["ignoreOffer"] = not peer["polite"] and offerCollision

            if(peer["ignoreOffer"]):
                if(self.verbose):
                    print("Peer offer was ignore because we are impolite")
                return

            if(offerCollision):
                # not working in wrtc node.js
                await peerConnection.setLocalDescription({"type": "rollback"})
                # not working in wrtc node.js
                await peerConnection.setRemoteDescription(rtc_decription)
            else:
                # Otherwise there are no collision and we can take the offer as our remote description
                await peerConnection.setRemoteDescription(rtc_decription)
                if(peer.get('remoteStream')):
                    print("this is the remote stream",peer.get('remoteStream'))
                    await peer.get('remoteStream').start()

                # if(self.options.get('enableRemoteStream')):
                    # await peer.get('remoteStream').start()
            if(description["type"] == 'offer'):
                if(self.options.get('enableLocalStream')):
                    self.update_local_streams(peer)
                await peerConnection.setLocalDescription(await peerConnection.createAnswer())
                if(self.verbose):
                    print("Sending answer to {}".format(peer["peerId"]))
                await self.signaling_channel.send_to(
                    peer["peerId"], {"action": "sdp", "sdp": {
                        "sdp": peerConnection.localDescription.sdp,
                        "type": peerConnection.localDescription.type}})

        except:
            print("Something related to update_session_description went wrong")

    async def update_ice_candidate(self, peer, candidate):
        """
        The logic to update the ICE Candidates during negotiation

        Args:
            peer (dict): Data about the peer. Containes ["peerId"(string), "peerId" (string),"peerType" (string),"polite" (bool),"rtcPeerConnection" (RTCPeerConnection),"dataChannel" (RTCDataChannel),"makingOffer" (bool),"ignoreOffer" (bool),"isSettingRemoteAnswerPending" (bool)]
            candidate (dict): The ICE Candidate from peer
        """
        peerConnection = peer["rtcPeerConnection"]
        try:
            if(candidate != None):  # address == ip
                rtc_candidate = RTCIceCandidate(candidate.get("component"),
                                                candidate.get("foundation"),
                                                candidate.get("address"),
                                                candidate.get("port"),
                                                candidate.get("priority"),
                                                candidate.get("protocol"),
                                                candidate.get("type"),
                                                candidate.get(
                                                    "relatedAddress"),
                                                candidate.get("relatedPort"),
                                                candidate.get("sdpMid"),
                                                candidate.get("sdpMLineIndex"),
                                                candidate.get("tcpType"))
                await peerConnection.addIceCandidate(rtc_candidate)
        except:
            if(not peer["ignoreOffer"]):
                print("Something related to addIceCandidate went wrong")

    # def update_remote_streams(self, peer):
        # print("Updating remote stream")
        # peerConnection = peer.get('rtcPeerConnection')
        # peer['remoteStream'] = MediaRecorder(peer.get("peerId")+'_video.mp4')
        # @peerConnection.on("track")
        # def on_track(track):
            # print("Receiving %s" % dir(track))
            # peer.get('remoteStream').addTrack(track)
            # remote_relay.subscribe(track)
            # frame = await track.recv()
            # print(dir(frame))
            # print(frame.to_ndarray())
            # cv2.imshow("video", frame)
            # print(frame)


    def update_local_streams(self, peer):
        print("Updating local streams")
        peerConnection = peer.get('rtcPeerConnection')

        # Create local track
        # options = {"framerate": "30", "video_size": "640x480"}
        options = {"video_size": "640x480"}

        if platform.system() == "Darwin":
                webcam = MediaPlayer(
                "default:none", format="avfoundation", options=options
            )
        elif platform.system() == "Windows":
            webcam = MediaPlayer(
                "video=Integrated Camera", format="dshow", options=options
            )
        else:
            webcam = MediaPlayer("/dev/video0", format="v4l2", options=options)
        # relay = MediaRelay()
        # video = relay.subscribe(webcam.video)
        audio = None
        # print("looking for trans")
        for t in peerConnection.getTransceivers():
            print(t)
            if t.kind == "audio" and audio:
                peerConnection.addTrack(audio)
            elif t.kind == "video" and webcam.video:
                peerConnection.addTrack(webcam.video)
        # peerConnection.addTrack(webcam.video)


    async def remove_peer(self, peer):
        """
        Closes all connections and removes peer from connection. Fired when the peer has left the signaling server or when a close action is sent.

        Args:
            peer (dict): Data about the peer. Containes ["peerId"(string), "peerId" (string),"peerType" (string),"polite" (bool),"rtcPeerConnection" (RTCPeerConnection),"dataChannel" (RTCDataChannel),"makingOffer" (bool),"ignoreOffer" (bool),"isSettingRemoteAnswerPending" (bool)]
        """
        if(peer.get('dataChannel')):
            peer["dataChannel"].close()
        await peer["rtcPeerConnection"].close()
        del self.peers[peer["peerId"]]
        if(self.verbose):
            print("Connection with {} has been removed".format(peer["peerId"]))

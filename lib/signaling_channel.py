import asyncio
import socketio


class SignalingChannel():

    def __init__(self, peer_id, peer_type, signaling_server_url, token, verbose=True):
        """
        Class representing the isgnaling channel used to establish WebRTC communication in a meshed network of peers using socket.io. Connects to the signaling server and enables messaging to all peers or only one peer.

        Args:
            peer_id (string): Id to identify the peer in the signaling server and WebRTC communication. Must be unique for each peer, otherwise a Uniqueness error will occur.
            peer_type (string): What of peer this is. Fro exmaple it can be "admin", "vehicle", "controltower" or "robot" depeding on your own application.
            signaling_server_url (string): URL to the signaling server
            token (string): Token used to authenticate the connection to the signaling server
            verbose (bool, optional): If True the channel will print its when status when event occur. Defaults to True.
        """
        self.peer_id = peer_id
        self.peer_type = peer_type
        self.signaling_server_url = signaling_server_url
        self.token = token
        self.verbose = verbose
        self.socket = socketio.AsyncClient(
            reconnection=True  # Enables auto reconnection to server, this can occur when for example the host server disconnects. When set to true, the client would keep trying to reconnect
        )
        self.init_event_listeners()

    async def connect(self):
        """
        Establishes a connection to the signaling server.
        """
        try:
            await self.socket.connect(self.signaling_server_url,
                                      auth={"token": self.token})
            await self.socket.wait()
        except:
            pass

    async def send(self, payload):
        """
        Sends a message to all other peers connected to the signaling server

        Args:
            payload (dict): Data to send to all peers
        """
        await self.socket.emit("message", {"from": self.peer_id, "target": "all", "payload": payload})

    async def send_to(self, targetPeerId, payload):
        """
        Sends a message to a specific peer in the channel, based on their peer_id

        Args:
            targetPeerId (string): ID of the peer who should recieve the message
            payload (dict): Data to send to the peer
        """
        await self.socket.emit("messageOne", {"from": self.peer_id, "target": targetPeerId, "payload": payload})

    async def disconnect(self):
        """
        Disconnects peer from the signaling server.
        """
        if(self.socket):
            await self.socket.disconnect()

    def init_event_listeners(self):
        """
        Initializes all the events listeners related to the signaling channel.
        """
        socket = self.socket

        @socket.on('connect')
        async def on_connect():
            if(self.verbose):
                print("Connected to signaling server with id",
                      self.socket.sid)
            await self.socket.emit("ready", (self.peer_id, self.peer_type))

        @socket.event
        def disconnect():
            print("Disconnected from server")

        @socket.event
        def connect_error(data):
            print("The connection failed!", data)

        @socket.event
        def reconnect():
            if(self.verbose):
                print("Reconnected to signaling server.")

        @socket.event
        def message(message):
            if(self.verbose):
                print('Got message with:', message)

        @socket.on('uniquenessError')
        def on_uniqueness_error(error):
            print('UniquenessError:', error['message'])

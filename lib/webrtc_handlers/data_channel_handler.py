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

    @channel.on("open")
    def on_open():
        print("Data channel with", peer_id, "is open")
        channel.send("Hello from {}".format(our_peer_id))

    @channel.on("close")
    def on_close():
        print("Data channel with", peer_id, "has been closed")

    @channel.on("message")
    def on_message(message):
        print(peer_id, "says:", message)

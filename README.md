# Simple WebRTC Python Client

WebRTC is an evolving technology for peer-to-peer communication on the web. This repository demonstrates how this technology can be used to establish a peer connection from a Python instance. The networking topology is based on a [meshed network](https://webrtcglossary.com/mesh/). Any successful WebRTC connection requires a signaling server for the peers to exchange ICE candidates and session description protocol (SDP). The WebRTC client in this repository is compatible with the signaling server created in the following [repository](https://github.com/aljanabim/simple_webrtc_signaling_server).

#### Table of Contents

-   [Installation](#Installation)
-   [Usage](#Usage)
    -   [Development](#Development)
    -   [Production](#Production)
-   [Examples](#Examples)
    -   [Terminal Chat App](#terminal-chat-app)
    -   [ROS Client App](#ros-client-app)
-   [Implement Your Own Logic](#implement-your-own-logic)
    -   [Overall app logic](#overall-app-logic)
    -   [Utilize the WebRTC Data Channel](#utilize-the-webrtc-data-channel)
-   [Resources](#Resources)
-   [To Do](#to-do)

## Installation

We recommend using [Conda or (Miniconda)](https://conda.io/projects/conda/en/latest/user-guide/install/index.html#installing-conda-on-a-system-that-has-other-python-installations-or-packages) to manage the packages and versions of this WebRTC Python Client. You are free to choose your version manager of choice (eg. pyenv or plain pip), in that case have a look at [environment.yml](./environment.yml) to see what packages you need to install. Below are the installation instructions using Miniconda.

```bash
git clone https://github.com/aljanabim/simple_webrtc_python_client.git
cd simple_webrtc_python_client
conda env create -f environment.yml
conda activate webrtc
```

## Usage

This client works out of the box with the signaling server created in the [Simple WebRTC Signaling Server](https://github.com/aljanabim/simple_webrtc_signaling_server) repository. Make sure you have a running local or deployed instance of the signlaing server before proceeding. You can use the signaling server locally, to play with the client logic and the examples, or deploy it on the web using your deployment service of choice (eg. Heroku, GCP, AWS, Azure, Vercel, Netlify, etc.).

### Development

For **development** make sure to update the environment variables, in [/config/dev.env](/config/dev.env), according to the configuration of your signaling server. Then run:

```bash
python main.py dev [--id]
```

For example, `python main.py --dev --id=vehicle1`, where `--id` denotes the peer ID to use for the client. This ID must be **unique** for each peer. If not specified, the machine hostname will be used.

Once you have everything up and running it is time to either play with the [examples](#Examples) or to [Implement your own logic](#implement-your-own-logic)

### Production

For **production** make sure to update the environment variables, in [/config/dev.env](/config/prod.env), according to the configuration of your signaling server. Then run:

```bash
python main.py [--id]
```

For example, `python main.py --id=vehicle1`, where `--id` denotes the peer ID to use for the client. This ID must be **unique** for each peer. If not specified, the machine hostname will be used.

Once you have everything up and running it is time to either play with the [examples](#Examples) or to [Implement your own logic](#implement-your-own-logic)

### Issues so far

When a Node.js peer (using wrtc) is polite (reciever), all works well. But when the Node.js peer is the offerer (we are polite), the data channel never opens. This issue doesn't occur when Python clients are connecting to each other

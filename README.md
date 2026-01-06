# carvi_rsd
A Rust wrapper for Robosense rs_driver

This crate provides Rust bindings for the Robosense rs_driver C++ library, enabling seamless integration of Robosense LiDAR devices into Rust applications. It leverages the power of Rust's safety and concurrency features while utilizing the robust capabilities of the rs_driver library.

> [!WARNING]
> This crate is still in early development and is used primarily for CARVI's internal projects and as such may not implement all features of the underlying C++ library. You are welcome to contribute or open issues for missing features.
> At this time, we only tested with Robosense Airy LiDAR.

## Build Instructions
This crate depends on the [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver) repository. And so you need to make sure that you install rs_driver dependency before building this crate.

### Linux
If you want to enable PCAP playback/parsing support, you must install libpcap.

```sh
sudo apt-get install libpcap-dev libeigen3-dev libboost-dev libpcl-dev
```

### Windows

Building on Windows supports two modes:

- **Without PCAP support (default)**: no Npcap SDK required.
- **With PCAP support** (enable the `pcap` feature): requires the Npcap runtime + SDK.

To enable PCAP support, build with the `pcap` feature.

1. Install the Npcap runtime: https://npcap.com/#download
	- During install, enable **"Install Npcap in WinPcap API-compatible Mode"**.

2. Install the Npcap SDK (separate download on the same page): https://npcap.com/#download
	- Extract the SDK to one of these locations:
	  - `C:\Program Files\Npcap SDK` (recommended)
	  - `C:\npcap-sdk`
	  - or any custom location (see `NPCAP_SDK` below)

	The SDK should contain:

	```text
	Npcap SDK/
	|-- Include/
	|   `-- pcap.h
	`-- Lib/
	    `-- x64/
	        |-- wpcap.lib
	        `-- Packet.lib
	```

3. If you used a custom SDK location, set `NPCAP_SDK` to the SDK root (the folder containing `Include/` and `Lib/`):

	```powershell
	$env:NPCAP_SDK = "C:\path\to\Npcap SDK"
	```

4. Build:

	```sh
	cargo build
	```

If you see "pcap.h file not found", it means the SDK headers aren't being found. Ensure the SDK is installed, and that `NPCAP_SDK` points to the SDK root.

#### Enabling/disabling PCAP support

- Enable PCAP support:

	```sh
	cargo build --features pcap
	```

- PCAP support is **disabled by default**. To build without PCAP support, just build normally:

	```sh
	cargo build
	```

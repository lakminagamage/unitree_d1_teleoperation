# D1 Arm Development Setup

Tested on Ubuntu 20.04. The D1 arm communicates over Ethernet using DDS (Data Distribution Service) via the `unitree_sdk2` library.

---

## Requirements

- Ubuntu 20.04
- cmake >= 3.5
- g++ with C++17 support
- libncurses5-dev
- unitree_sdk2 (installed system-wide)

```bash
sudo apt install cmake build-essential libncurses5-dev
```

---

## Install unitree_sdk2

Follow the official Unitree SDK2 installation guide to install `unitree_sdk2` to `/usr/local` before proceeding.

Verify installation:

```bash
ls /usr/local/include/ddscxx
ls /usr/local/include/iceoryx
```

---

## Network Setup

Connect the D1 arm via Ethernet. The arm's default IP is `192.168.123.100`.

Set your machine's Ethernet interface to a static IP in the same subnet, e.g. `192.168.123.12/24`.

Find your Ethernet interface name:

```bash
ip addr
```

Look for the interface with `192.168.123.x` — it will look something like `enx9c69d328e892` or `eth0`.

Verify connectivity:

```bash
ping 192.168.123.100
```

---

## Build

```bash
cd d1_sdk
mkdir build && cd build
cmake .. && make
```

This builds all example programs and the joint controller into the `build/` directory.

---

## Programs

| Binary | Description |
|---|---|
| `arm_zero_control` | Return arm to zero position |
| `joint_angle_control` | Move a single joint to a fixed angle |
| `multiple_joint_angle_control` | Move all joints simultaneously |
| `joint_enable_control` | Enable or disable all motors |
| `get_arm_joint_angle` | Read and print current joint angles |
| `d1_joint_controller` | Interactive TUI to control joints with sliders |

All programs require the network interface to be passed in `Init()`. If you need to change the interface, update the `ChannelFactory::Instance()->Init(0, "your_interface")` line in the relevant source file and rebuild.

---

## Running the Joint Controller

```bash
cd d1_sdk/build
./d1_joint_controller
```

| Key | Action |
|---|---|
| Up / Down | Select joint |
| Left / Right | Adjust angle by 1 degree |
| [ / ] | Adjust angle by 10 degrees |
| Space | Send all joints at once |
| e | Enable all motors |
| d | Disable (release) all motors |
| z | Return to zero position |
| q | Quit |

Always press `e` to enable motors before sending joint commands.

---

## Joint Limits

| Joint | Range |
|---|---|
| J0 - Base Rotation | -135 to +135 deg |
| J1 - Shoulder | -90 to +90 deg |
| J2 - Elbow | -90 to +90 deg |
| J3 - Forearm Roll | -135 to +135 deg |
| J4 - Wrist Pitch | -90 to +90 deg |
| J5 - Wrist Roll | -135 to +135 deg |
| J6 - Claw | 0 to 65 mm |

---

## Firmware Upgrade (IAP)

Connect arm via Ethernet, then open a browser and go to:

```
http://192.168.123.100:8080
```

Upload the firmware package provided by Unitree.
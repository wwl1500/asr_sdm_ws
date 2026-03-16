# [WIP]asr_sdm_radxa_ws
Workspace of Radxa Zero for Amphibious Snake-like Robot with Screw-drive Mechanism

## Environment Setup

### .bash.rc
```sh
alias ls='ls --color=auto'  # Linux
alias ls='ls -G'            # macOS
export CLICOLOR=1           # macOS 启用颜色
export LSCOLORS="ExGxFxdaCxDaDahbadacec"  # macOS 颜色方案
export TERM=xterm-256color
export PS1='\[\e[1;32m\]\u@\h:\w\$\[\e[0m\] '  # 绿色提示符
source /opt/ros/rolling/setup.bash
export ROS_DOMAIN_ID=176
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=/opt/ros/2/
```

### C Library for Linux Peripheral I/O (GPIO, LED, PWM, SPI, I2C, MMIO, Serial)
https://github.com/vsergeev/c-periphery

### How to grant user permissions to utilize the spidev port
```sh
ls -l /dev/spidev*
sudo groupadd spi
sudo usermod -aG spi $USER
sudo nano /etc/udev/rules.d/99-spi.rules
SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"

sudo groupadd gpio
sudo usermod -aG gpio $USER
sudo nano /etc/udev/rules.d/99-gpio.rules
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", MODE="0660", GROUP="gpio"

# Check the uart group
ls -l /dev/ttyS*
# You’ll see something like:
crw-rw---- 1 root dialout 4, 64 Sep  4 00:12 /dev/ttyS*
# In this example, the group is dialout. Suppose the group is dialout, run:
sudo usermod -aG dialout $USER
# Reboot ,then check with:
cat /dev/ttyS*

sudo nano /etc/udev/rules.d/99-uart.rules
SUBSYSTEM=="tty", GROUP="uart", MODE="0660"
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo reboot
```

### Add .dtbo filesm
```sh
sudo nano /etc/default/u-boot

# uncomment and modify the following lines
U_BOOT_UPDATE="true"
U_BOOT_FDT_OVERLAYS="rk3568-spi3-m1-cs0-mcp2515.dtbo rk3568-i2c4-m0.dtbo"
U_BOOT_FDT_OVERLAYS_DIR="/lib/firmware/6.1.0-1025-rockchip/device-tree/rockchip/overlay"
U_BOOT_SYNC_DTBS="true"

# update u-boot
sudo u-boot-update
```

### Driver installation
Driver files are all saved here.
```sh
/lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/spi/mcp251x.ko
/lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/dev/can-dev.ko
sudo insmod /lib/modules/6.1.0-1025-rockchip/kernel/drivers/net/can/dev/can-dev.ko
```

Add the drivers to automatic startup list.
```sh
sudo nano /etc/modules-load.d/modules-load.d
# add the following two lines
can-dev
mcp251x.ko
```

Check if the driver is installed successfully.
```sh
lsmod | grep spidev
dmesg | grep can
dmesg | grep mcp251
```

### Interface test
```sh
sudo ifconfig can0 txqueuelen 65536
sudo ip link set can0 up type can bitrate 500000
candump can0
ip link show
ip -details -statistics link show can0
```

## Dependencies

### glog
Install the official glog library. This is used instead of the `glog_vendor` package.
```sh
sudo apt-get update && sudo apt-get install -y libgoogle-glog-dev
```

## Repository Structure

```
asr_sdm_ws/
├── src/
│   ├── asr_sdm_launch/          # Launch files and configurations
│   │   ├── config/
│   │   └── launch/
│   ├── asr_sdm_msgs/            # Custom ROS2 message definitions
│   │   ├── asr_sdm_control_msgs/
│   │   ├── asr_sdm_hardware_msgs/
│   │   ├── asr_sdm_vio_msgs/
│   │   └── ...
│   ├── asr_sdm_sensors/         # Sensor driver packages
│   │   ├── asr_sdm_camera_realsense_d405/
│   │   ├── asr_sdm_imu_hiwonder_10axis/
│   │   └── asr_sdm_imu_wheeltec_n100/
│   └── asr_sdm_universe/        # Core robot software
│       ├── common/              # Shared libraries (geometry, data structures)
│       ├── control/             # Controllers and dynamic models
│       ├── perception/          # Vision and odometry
│       └── robot/               # Hardware interface, description, teleop
├── datasheet/                   # Hardware datasheets
├── block_diagram.drawio.svg
├── board_pin_map.drawio.svg
├── comm_protocol.md
└── README.md
```

### How to reference folders and files

In GitHub issues, pull requests, and Markdown files, use the following conventions to reference folders and files in this repository:

**Inline code** — wrap a path in backticks to display it as a fixed-width path:
```
`src/asr_sdm_launch/launch/`
`src/asr_sdm_universe/control/asr_sdm_controller/`
```

**Markdown hyperlink** — link directly to a file or folder on GitHub:
```markdown
[asr_sdm_controller](src/asr_sdm_universe/control/asr_sdm_controller/)
[interface_launch.py](src/asr_sdm_launch/launch/interface_launch.py)
```

**Absolute GitHub URL** — use the full URL when linking from outside this repository (replace `<branch>` with your branch name, e.g. `main`):
```
https://github.com/wwl1500/asr_sdm_ws/tree/<branch>/src/asr_sdm_universe/control/
https://github.com/wwl1500/asr_sdm_ws/blob/<branch>/src/asr_sdm_launch/launch/interface_launch.py
```

> **Tip:** Use `tree/` in the URL to link to a **folder**, and `blob/` to link to a **file**.

## ROS

### Source code compilation
```sh
colcon build --symlink-install --parallel-workers 2
```


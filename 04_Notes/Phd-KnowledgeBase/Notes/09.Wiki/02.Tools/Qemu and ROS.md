# Qemu and ROS

## Qemu vm installation

https://metal.equinix.com/developers/guides/kvm-and-libvirt/

virsh

# ROS 2 humble installation

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Mvsim

https://mvsimulator.readthedocs.io/en/latest/install.html#from-ros1-or-ros2-repositories


The main objective is to get virtualized qemu rpi4 devices
each device shall be run in PXE mode
Boot up should occur to assign each device a unique Ip address and hostname

### Link to build the rpi-k8-cluster

https://picluster.ricsanfre.com/

## Flashing software

https://github.com/hypriot/flash

Tools

- ubuntu core or ubuntu server 64 (consider using cloud-init)

### Example - ubuntu 22 server emulation

```sh
wget https://cdimage.ubuntu.com/releases/22.04/release/ubuntu-22.04-preinstalled-server-arm64+raspi.img.xz
xz -d ubuntu-22.04-preinstalled-server-arm64+raspi.img.xz
mkdir -p /mnt/rpi4
fdisk -l ubuntu-22.04-preinstalled-server-arm64+raspi.img
sudo mount -o offset=269484032 ubuntu-22.04-preinstalled-server-arm64+raspi.img /mnt/rpi4/

```
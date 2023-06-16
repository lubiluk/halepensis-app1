# Halepensis Docker
This Dockerfile defines an image with Ubuntu 20.04 (bionic) and ROS Noetic. It builds Halepensis from a local copy.

For testing you can run `scripts/run_halepensis_example.sh` which downloads an example rosbag, starts a tmux session in which the rosbag is played and halepensis is launched. 

## Requirements
- [docker](https://docs.docker.com/install/)
- [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)
- [docker-compose](https://docs.docker.com/compose/install/)

To allow the GUI for, e.g., rviz to work, we require X11 forwarding. Note that this alters X11's access protection and
therefore should be revoked after usage. We will disable X authentication for the `docker` group. 
- Add your user to the `docker` group (once): `usermod -aG docker ${USER}`
- Disable X authentication for the `docker` group: `xhost +local:docker`
- Enable X authentication again (afterwards): `xhost -local:docker`

Tested on Ubuntu 18.04 with NVIDIA driver version 510.

## Usage
- Disable X authentication (as above). Do not forget to enable it again once you are done.
- To (re)build the container: `halepensis-app1/docker$: docker-compose build halepensis`
- To run the container: `halepensis-app1/docker$: docker-compose run halepensis`

Thanks to [@dornik](https://github.com/dornik) for the original Dockerfile. 
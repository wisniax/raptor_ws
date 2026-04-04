# Setting things up
## Cloning
`git clone git@github.com:wisniax/raptor_ws.git`

## Get the submodules
`git submodule update --init`

## Build the container
`docker compose build`
> To enable passwordless ssh into rex: \
> Copy your public ssh files to .devcontainer/public_keys.d \
> **Note!** Do this step before building the container or rebuild after!

## Generate TLS keys, certificates and credentials for Mosquitto
A helper script is included, which allows to easily generate a CA, server certificate, server keys, and an MQTT credentials file for Mosquitto; it also automatically sets the correct file permissions and ownership, and copies the files into the right locations. `openssl` package, Docker, and sudo access are required to run this script. The syntax is as follows:
`./mqtt_certs_gen.sh <CA Common Name> <Server SAN> <MQTT Username> <MQTT Password>`
where `CA Common Name` should be a name which identifies the Certificate Authority, and `Server SAN` should be the **IP address** (NOT DNS!) by which clients should be able to reach the MQTT broker (`localhost` is already included for in-docker access). Example:
`./mqtt_certs_gen.sh RaptorsCA 192.168.1.20 raptors changeme`
(for local development, `127.0.0.1` can be used)

**Remember to set the correct MQTT username+password in mqtt_bridge (`mqtt_bridge_node.cpp`), and create a `.env` file with `MQTT_USERNAME` and `MQTT_PASSWORD` variables in the root of this repository (to override the defaults for Mosquitto healthcheck). The pre-defined values in both those places are `raptors`/`changeme`. Also, for proper security you should enable server cert auth by setting `mqtt_enable_server_cert_auth` to `true` in `mqtt_bridge.yaml` launch file.** Of course, after changes to the cpp and yaml file, the mqtt_bridge package has to be re-built (see *Build the repo* section below).

**Beware that a production setup definitely requires a more thorough SSL configuration than this script can provide!**

## Start the container
`docker compose up -d`

If docker compose complains about missing files to mount, see the section above - the TLS and credentials stuff **need** to be present in order for the containers to start.

### To stop the container
`docker compose down`

### Environment variables
- `ROS_ENABLE_AUTOSTART` *(somewhat works)*
    - **`0`** (default) - ROS autostart disabled
    - `1` - ROS autostart enabled
- `ROS_BUILD_ON_STARTUP` *(to be implemented)*
    - `never` - No build on startup. \
    No boot performance impact.
    - **`on-failure`** (default) - Try build only once when *ROS_ENABLE_AUTOSTART* is enabled and the startup failed (any error). 
    On successful build, startup will be retried. \
    High boot performance impact, but only when rebuild is needed (typically only once).
    - `always` - Always build on startup. \
    Very high boot performance impact.
- `ROS_ENABLE_CAN_BRIDGE` *(to be implemented)*
    - **`0`** (default) - Do not trigger can bridge to set-up, nor wait for it
    - `1` - Trigger *host* to set-up can bridge. Requires additional configuration on host side (extra services) to work. \
    When configured properly: low boot performance impact (0.4 sec). Will delay startup by 5 sec when this is enabled but not configured!

# Container features
## SSH into the container
`ssh rex@[ip-here] -p 2122`
e.g. `ssh rex@localhost -p 2122`

## Build the repo
> Obviously only works from inside the container ;)

`cd raptor_ws`

`colcon build --symlink-install`
> If colcon fails with permission denied run:
`chmod g+rw -R /home/rex/raptor_ws`

## Using the rex service
### Stop the rex main ros program:
`sudo service rex stop`

### Start the rex main ros program:
`sudo service rex start`

### Tail logfile
`sudo service rex logs`

## Setting up can bridge to autostart properly
Can bridge setup is a feature related to `ROS_ENABLE_CAN_BRIDGE` docker flag. 
For can interface to work properly inside docker that does not have *network: "host"* defined we need to get creative and reach for... virtual interfaces!. Specifically, we use vxcan for this. This is also included in the Linux kernel. Vxcan allows to create virtual CAN interfaces and to set up tunnels between them. This makes it relatively easy to communicate across network namespaces. For example, between two or more Docker containers. ([source](https://www.systec-electronic.com/en/demo/blog/article/news-socketcan-docker-the-solution))

> **Warn!** Hot-swapping physical can-interfaces is currently **NOT** supported. *(TODO Task)*

### Physical can interface auto bringup
TBD

### Virtual can bridge setup
First make sure your shell is in *raptor_ws* directory.

#### Now copy necessary services and executables to system:
```
sudo cp .devcontainer/services/can-bridge.* /etc/systemd/system/
sudo cp .devcontainer/services/can-bridge-setup.sh /usr/local/bin/
```
> **Note!** Those services are responsible for detecting a file in raptor_ws (.path one) which triggers (together with .service file) a can bridge creation.

#### Edit path in `can-bridge.path` service to `raptor_ws` directory
```
sudo vim /etc/systemd/system/can-bridge.path
```
Change `PathExists=/path/to/raptor_ws/.can_bridge_rex_waiting` to correct `raptor_ws` path for your system.

#### Then edit path in `can-bridge.service` service to `raptor_ws` directory
```
sudo vim /etc/systemd/system/can-bridge.service
```
Change `ExecStartPost=/bin/rm -f /path/to/raptor_ws/.can_bridge_rex_waiting` to correct `raptor_ws` path for your system.

#### Setup correct permissions for executable
```
sudo chmod +x /usr/local/bin/can-bridge-setup.sh
```

#### Reload `systemd` and start the service
```
sudo systemctl daemon-reload
sudo systemctl enable can-bridge.path
sudo systemctl start can-bridge.path
```
> **Note!** Do not skip *.path* above. This results in improper setup!

#### Debug can-bridge action
```
sudo systemctl status can-bridge.service
```
```
sudo journalctl -u can-bridge.service -b
```

# Build using GH actions runner
Make `.secrets` file in project directory with contents:
```
DOCKERHUB_TOKEN=<docker-hub-token>
DOCKERHUB_USERNAME=<username>
GITHUB_TOKEN=<github-token>
```
then run
```
act
```
> Note! This will build the container locally and push it to dockerhub!

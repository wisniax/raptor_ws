# Setting things up

## Host prerequisites
- Docker
- Docker Compose
- Git
- OpenSSL
- Bash

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
A helper script is included, which allows to easily generate a CA, server certificate, server keys, and an MQTT credentials file for Mosquitto; it also automatically sets the correct file permissions and ownership, and copies the files into the right locations. `openssl` package, Docker, sudo access, and belonging to the `docker` group are required to run this script. The syntax is as follows:
`./mqtt_certs_gen.sh <CA Common Name> <Server SAN> <MQTT Username> <MQTT Password>`
where `CA Common Name` should be a name which identifies the Certificate Authority, and `Server SAN` should be the **IP address** (NOT DNS!) by which clients should be able to reach the MQTT broker (`localhost` is already included for local access). Example:
`./mqtt_certs_gen.sh RaptorsCA 192.168.1.20 raptors changeme`
(for local development, `127.0.0.1` can be used)

**Remember to set the correct MQTT username+password in mqtt_bridge (`MqttBridge.hpp`), and create a `.env` file with `MQTT_USERNAME` and `MQTT_PASSWORD` variables in the root of this repository (to override the defaults for Mosquitto healthcheck). The pre-defined values in both those places are `raptors`/`changeme`. Also, for proper security you should enable server cert auth by setting `mqtt_enable_server_cert_auth` to `true` in `mqtt_bridge.yaml` launch file.** Of course, after changes to the hpp and yaml file, the mqtt_bridge package has to be re-built (see *Build the repo* section below).

**Beware that a production setup definitely requires a more thorough TLS/SSL configuration than this script can provide!**

## Start the ROS and Mosquitto containers
`docker compose up -d`

If docker compose complains about missing files to mount, see the section above - the TLS and credentials stuff **need** to be present in order for the containers to start.

### To stop the containers
`docker compose stop`

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

# Container features
## SSH into the container
`ssh rex@[ip-here] -p 2122`
e.g. `ssh rex@localhost -p 2122`

> default password: `changeme`

## Build the repo
> The commands below should be executed inside the ROS container.

`cd raptor_ws`

`colcon build --symlink-install`
> If colcon fails with permission denied run:
`chmod g+rw -R /home/rex/raptor_ws`

### Source the workspace

`source install/setup.bash`

### Run integration tests
> NOTE: Set the correct MQTT connection parameters in `mqtt_bridge_tests.cpp`. Obviously, Mosquitto must be running during the tests.

`colcon test`

## Using the rex service
### Stop the rex main ros program:
`sudo service rex stop`

### Start the rex main ros program:
`sudo service rex start`

### Tail logfile
`sudo service rex logs`

## Setting up CAN interface to autostart properly
Example for NixOS, using systemd-networkd:
```
  systemd.network.enable = true;

  systemd.network.networks."80-can" = {
    matchConfig.Name = "can0";
    networkConfig = { };
    extraConfig = ''
      [Link]
      RequiredForOnline=no

      [CAN]
      BitRate=500000
      RestartSec=100ms
    '';
  };
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

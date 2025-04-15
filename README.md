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


## Start the container
`docker compose up -d`

### To stop the container
`docker compose down`

# Features
## SSH into the container
`ssh rex@[ip-here] -p 2122`
e.g. `ssh rex@localhost -p 2122`

## Build the repo
> Obviously only works from inside the container ;)

`cd raptor_ws`

`colcon build --symlink-install`
> If colcon fails with permission denied run:
`chmod a+rw -R /home/rex/raptor_ws`

## Using the rex service
### Stop the rex main ros program:
`sudo service rex stop`

### Start the rex main ros program:
`sudo service rex start`

### Tail logfile
`sudo service rex logs`
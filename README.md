# Setting things up
## Cloning
`git clone git@github.com:wisniax/raptor_ws.git`
## Get the submodules
`git submodule update --init`
## Build the container
`docker compose build`
## Start the container
`docker compose up -d`

### To stop the container
`docker compose down`

# Features
## SSH into the container
`ssh rex@[ip-here] -p 2122`
e.g. `ssh rex@localhost -p 2122`

## Build the repo
`cd raptor_ws`

`colcon build --symlink-install`
> If colcon fails with permission denied run:
`chmod a+rw -R /home/rex/raptor_ws`
## Stop the rex main ros program:
`sudo service rex stop`

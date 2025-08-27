#!/bin/bash

# Check that 4 parameters were passed
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <CA Common Name> <Server SAN> <MQTT Username> <MQTT Password>"
    exit 1
fi

CA_CN="$1"
SERVER_SAN="$2"
MQTT_USERNAME="$3"
MQTT_PASSWORD="$4"

# Clear any existing certs (if they exist)
sudo rm -r .devcontainer/mqtt-server-certs/
mkdir .devcontainer/mqtt-server-certs/

# Adapted from: https://www.gravio.com/en-blog/tutorial-how-to-set-up-a-mosquitto-mqtt-broker-securely----using-client-certificates
openssl genrsa -out .devcontainer/mqtt-server-certs/ca.key 2048
openssl req -new -x509 -days 3650 -key .devcontainer/mqtt-server-certs/ca.key -out .devcontainer/mqtt-server-certs/ca.crt -subj "/CN=${CA_CN}"
openssl genrsa -out .devcontainer/mqtt-server-certs/server.key 2048
openssl req -new -key .devcontainer/mqtt-server-certs/server.key -out .devcontainer/mqtt-server-certs/server.csr -addext "subjectAltName = DNS:localhost,IP:${SERVER_SAN}" -subj "/CN=mosquitto"
openssl x509 -req -in .devcontainer/mqtt-server-certs/server.csr -CA .devcontainer/mqtt-server-certs/ca.crt -CAkey .devcontainer/mqtt-server-certs/ca.key -CAcreateserial -copy_extensions copy -out .devcontainer/mqtt-server-certs/server.crt -days 3650

# Remove Certificate Signing Request (no longer needed)
rm .devcontainer/mqtt-server-certs/server.csr

# Permissions and ownership
sudo chown 1883:1883 .devcontainer/mqtt-server-certs/ca.crt
sudo chown 1883:1883 .devcontainer/mqtt-server-certs/ca.key
sudo chown 1883:1883 .devcontainer/mqtt-server-certs/server.crt
sudo chown 1883:1883 .devcontainer/mqtt-server-certs/server.key
sudo chmod 0700 .devcontainer/mqtt-server-certs/ca.key
sudo chmod 0700 .devcontainer/mqtt-server-certs/server.key

# Copy CA cert for use with mqtt_bridge (reason: different permissions and ownership)
sudo cp -f .devcontainer/mqtt-server-certs/ca.crt .devcontainer/ca.crt

# More permissions and ownership...
sudo chown 1001:1000 .devcontainer/ca.crt
sudo chmod 0744 .devcontainer/ca.crt

# Mosquitto password file
sudo rm .devcontainer/mosquitto_passwd
docker run --rm eclipse-mosquitto sh -c "mosquitto_passwd -b -c /tmp/mosquitto_passwd ${MQTT_USERNAME} ${MQTT_PASSWORD} && cat /tmp/mosquitto_passwd" > .devcontainer/mosquitto_passwd
sudo chown 1883:1883 .devcontainer/mosquitto_passwd
sudo chmod 0700 .devcontainer/mosquitto_passwd

#!/bin/bash

# Check that 2 parameters were passed
if [ "$#" -ne 2 ]; then
    echo "Usage: $0 <CA Common Name> <Server SAN>"
    exit 1
fi

CA_CN="$1"
SERVER_SAN="$2"

# Clear any existing certs (if they exist)
sudo rm -r .devcontainer/mqtt-server-certs/
mkdir .devcontainer/mqtt-server-certs/

# Adapted from: https://www.gravio.com/en-blog/tutorial-how-to-set-up-a-mosquitto-mqtt-broker-securely----using-client-certificates
openssl genrsa -out .devcontainer/mqtt-server-certs/ca.key 2048
openssl req -new -x509 -days 3650 -key .devcontainer/mqtt-server-certs/ca.key -out .devcontainer/mqtt-server-certs/ca.crt -subj "/CN=${CA_CN}"
openssl genrsa -out .devcontainer/mqtt-server-certs/server.key 2048
openssl req -new -key .devcontainer/mqtt-server-certs/server.key -out .devcontainer/mqtt-server-certs/server.csr -addext "subjectAltName = DNS:mosquitto,IP:${SERVER_SAN}" -subj "/CN=mosquitto"
openssl x509 -req -in .devcontainer/mqtt-server-certs/server.csr -CA .devcontainer/mqtt-server-certs/ca.crt -CAkey .devcontainer/mqtt-server-certs/ca.key -CAcreateserial -copy_extensions copy -out .devcontainer/mqtt-server-certs/server.crt -days 3650

# Remove Certificate Signing Request (no longer needed)
rm .devcontainer/mqtt-server-certs/server.csr

# Permissions and ownership
sudo chown -R 1883:1883 .devcontainer/mqtt-server-certs/
sudo chmod -R 0700 .devcontainer/mqtt-server-certs/

# Copy CA cert to mqtt_bridge
sudo cp -f .devcontainer/mqtt-server-certs/ca.crt src/mqtt_bridge/ssl-certs/

# More permissions and ownership...
sudo chown 1001:1000 src/mqtt_bridge/ssl-certs/ca.crt
sudo chmod 0744 src/mqtt_bridge/ssl-certs/ca.crt

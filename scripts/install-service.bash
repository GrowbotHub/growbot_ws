#!/bin/bash

SERVICE_NAME=growbothub-ros.service
SERVICE_PATH=/etc/systemd/system/${SERVICE_NAME}

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
EXE_PATH="${DIR}/start-ws.bash"

# Populate service file
sudo bash -c "cat > ${SERVICE_PATH}" << EOF
[Unit]
Description=GrowbotHub ROS
[Service]
User=pi
ExecStart=${EXE_PATH}
[Install]
WantedBy=multi-user.target
EOF

sudo chmod 664 ${SERVICE_PATH}
sudo systemctl daemon-reload
sudo systemctl enable ${SERVICE_NAME}
sudo systemctl start ${SERVICE_NAME}

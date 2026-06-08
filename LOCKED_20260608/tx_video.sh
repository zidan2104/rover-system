#!/bin/bash

WLAN="wlan0"
KEY="/etc/drone.key"
CHANNEL=7
PORT=5600
LOCK_PORT=5601          # uplink perintah LOCK (RF -> diteruskan ke K230)

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

K230_IP="192.168.88.1"
STREAM_PORT="8554"

cleanup() {
    echo -e "\n${RED}Stopping TX...${NC}"

    sudo pkill -f wfb_tx
    sudo pkill -f wfb_rx
    pkill -f ffmpeg

    exit 0
}

trap cleanup SIGINT SIGTERM

echo -e "${GREEN}[1/5] Stop old services${NC}"

sudo pkill -f wfb_tx
sudo pkill -f wfb_rx
pkill -f ffmpeg

echo -e "${GREEN}[2/5] Set monitor mode${NC}"

sudo ip link set $WLAN down
sudo iw dev $WLAN set type monitor
sudo ip link set $WLAN up
sudo iw dev $WLAN set channel $CHANNEL

echo -e "${GREEN}[3/5] Start wfb_tx (-p 0) video${NC}"

sudo wfb_tx -p 0 -u $PORT -K $KEY $WLAN &

echo -e "${GREEN}[4/5] Start UPLINK wfb_rx (-p 1) perintah LOCK -> K230${NC}"
# Terima perintah LOCK dari RF, teruskan langsung ke K230 (192.168.88.1:$LOCK_PORT)
sudo wfb_rx -p 1 -u $LOCK_PORT -c $K230_IP -K $KEY $WLAN &

sleep 1

echo -e "${GREEN}[5/5] Forward TCP stream -> UDP${NC}"

ffmpeg \
    -fflags nobuffer \
    -flags low_delay \
    -max_delay 0 \
    -i tcp://$K230_IP:$STREAM_PORT?tcp_nodelay=1 \
    -an \
    -c:v copy \
    -f mpegts \
    -flush_packets 1 \
    udp://127.0.0.1:$PORT?pkt_size=1316

cleanup
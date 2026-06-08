#!/bin/bash
WLAN="wlxf4ec3889c21c"
KEY="$HOME/wfb-ng/gs.key"
CHANNEL=7
PORT=5600
LOCK_PORT=5601          # uplink perintah LOCK (debug.py -> di sini)
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'
cleanup() {
    echo -e "\n${RED}Stopping RX...${NC}"
    sudo pkill -f wfb_rx
    sudo pkill -f wfb_tx
    exit 0
}
trap cleanup SIGINT SIGTERM
echo -e "${GREEN}[1/4] Clean old process${NC}"
sudo pkill -f wfb_rx
sudo pkill -f wfb_tx
echo -e "${GREEN}[2/4] Set monitor mode${NC}"
sudo ip link set $WLAN down
sudo iw dev $WLAN set type monitor
sudo ip link set $WLAN up
sudo iw dev $WLAN set channel $CHANNEL
echo -e "${GREEN}[3/4] Start UPLINK wfb_tx (-p 1) untuk perintah LOCK${NC}"
# Baca UDP 127.0.0.1:$LOCK_PORT (dari debug.py) -> pancarkan via RF radio-port 1
sudo wfb_tx -p 1 -u $LOCK_PORT -K $KEY $WLAN &
sleep 1
echo -e "${GREEN}[4/4] Start wfb_rx (-p 0) video${NC}"
echo -e "${BLUE}Video di GCS program. Tombol LOCK kirim perintah via uplink. Ctrl+C stop.${NC}"
sudo wfb_rx -p 0 -u $PORT -K $KEY $WLAN
cleanup

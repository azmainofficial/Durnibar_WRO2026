#!/bin/bash
echo 123 | sudo -S wpa_cli -i wlan0 select_network 1
sleep 10
echo 123 | sudo -S wpa_cli -i wlan0 status > /tmp/wpa_status.txt
echo 123 | sudo -S wpa_cli -i wlan0 select_network 0

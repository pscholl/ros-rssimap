#!/usr/bin/env bash

f=$(rosparam get /current_wifi_playback)
rostopic echo -p /rssi_pose > ${f/%.bag/.csv}


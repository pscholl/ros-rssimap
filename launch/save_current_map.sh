#!/usr/bin/env bash

f=$(rosparam get /current_wifi_playback)
rosrun map_server map_saver -f ${f/%.bag/}


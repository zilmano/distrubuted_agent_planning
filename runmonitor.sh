#!/bin/bash
roscore &
sleep 3
rosrun distributed_mapf monitor &
sleep 4
rosrun distributed_mapf central_clock


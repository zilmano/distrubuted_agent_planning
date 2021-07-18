#!/bin/bash
roscore &
sleep 2
rosrun distributed_mapf monitor &
sleep 4
rosrun distributed_mapf central_clock &
./src/distributed_mapf/bin/websocket &
sleep 10 
rosrun distributed_mapf clientsim 


#!/bin/bash
# usage: delayed_roslaunch.sh <seconds> <roslaunch args...>  (rosrun/node remap args like __name:= are dropped)
d=$1; shift
args=()
for a in "$@"; do [[ $a == __* || $a == _* ]] || args+=("$a"); done
sleep "$d"
exec roslaunch "${args[@]}"

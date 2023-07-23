#! /bin/bash

function killLoadProcesses() {
    for t in "${loadPIDS[@]}"
    do
        kill -15 $t
    done
    if [ $? -eq 0 ]
    then
        echo "load processes terminated successfully"
    else
        echo "something went wrong while terminating the load processes"
    fi
    exit 0
}

trap killLoadProcesses SIGINT


# variables
cpuCores=$(grep --count ^processor /proc/cpuinfo)
loadPIDS=()
i=0
# generate load depending on available cpuCores
while [ $i -lt $(($cpuCores * 8 / 10)) ]
do
    dd if=/dev/zero of=/dev/null &
    loadPIDS+="$! "
    ((i++))
done

# wait for user interaction
read
killLoadProcesses

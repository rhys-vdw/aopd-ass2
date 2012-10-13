#!/bin/bash

if [ -z "$1" ]
then
  fileName="config.properties"
else
  fileName="$1"
fi

echo "Running path planner on configuration file" "$fileName"
echo ================================================
echo "$fileName"
echo ================================================

# -Djava.library.path=/home/rhys/uni/aopd/ass/2/aopd-ass2
# -cp /home/rhys/uni/aopd/ass/2/aopd-ass2/lib/bin/agents Apparate

echo
java -Djava.library.path=lib/ -cp bin/:lib/JPathPlan-v1.7.jar:lib/Apparate-v3.1.jar:lib/MyCoolAgent.jar pplanning.simviewer.controller.Launcher $fileName -

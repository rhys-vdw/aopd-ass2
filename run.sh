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


echo
java -cp bin/:lib/JPathPlan-v1.7.jar:lib/Apparate-v3.0.jar:lib/MyCoolAgent.jar pplanning.simviewer.controller.Launcher $fileName -

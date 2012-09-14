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
java -ea -cp bin/:lib/JPathPlan-v1.6.jar:lib/Apparate-v2.8.jar pplanning.simviewer.controller.Launcher $fileName

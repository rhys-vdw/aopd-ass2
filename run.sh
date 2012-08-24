#!/bin/bash

if [ -z "$1" ]
then
  fileName="config.properties"
else
  fileName="$1"
fi

echo "Running path planner on configuration file" "$fileName"
echo ================================================
cat "fileName"
echo ================================================


echo
java -cp bin/:lib/JPathPlan-v1.5.jar:lib/Apparate-v2.6.jar pplanning.simviewer.controller.Launcher $fileName

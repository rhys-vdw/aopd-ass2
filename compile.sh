#!
clear
javac -cp lib/JPathPlan-v1.7.jar:lib/Apparate-v3.0.jar::. src/agents/AStar/*.java src/agents/PreferredOperator/*.java src/agents/*.java -d bin -Xlint:deprecation

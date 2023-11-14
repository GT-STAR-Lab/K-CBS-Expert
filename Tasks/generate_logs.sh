#!/bin/bash

#Make sure you've built KCBS before running this!!
KBS_EXECUTABLE="../build/bicycle"

# Check if the required number of arguments is provided
if [ $# -ne 5 ]; then
  #Do not include the .xml for task_name as this is used as the start of the task name and will have an id appended
  #E.g., 
  echo "Usage: $0 'map_name' 'task_name' 'number_of_agents' 'number_of_tasks' 'config'"

  exit 1
fi

MAP_NAME="$1"
TASK_NAME="$2"
NUMBER_OF_AGENTS="$3"
NUMBER_OF_TASKS="$4"
CONFIG="$5"

python3 generate_tasks.py --map $MAP_NAME --agents $NUMBER_OF_AGENTS --name $TASK_NAME --num_tasks $NUMBER_OF_TASKS

#cd $KBS_DIR
for ((id = 0; id < NUMBER_OF_TASKS; id++)); do
  TASK_FILE="${TASK_NAME}_${id}"
    "$KBS_EXECUTABLE" "$(pwd)/$TASK_FILE.xml" "$(pwd)/$TASK_FILE.txt" #"$CWD/$CONFIG"
done

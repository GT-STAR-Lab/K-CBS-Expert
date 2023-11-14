To use this with Het-MAMP, first create a grid map (example can be seen in Examples/grid_map.xml). Then run generate_logs.sh with the following:

bash generate_logs.sh <map_name> <task_name> <number_of_agents> <number_of_tasks> <config>

Where <map_name> is the grid map created, task name is the name of the task you are making (do not include .xml at the end of this), numer of agents is how many agents are navigating in a tasks, number of tasks is the number of MAPF tasks to be randomly created for the map, and config is the general config file you're using (an example of this is in Examples/config.xml).

For example:

bash generate_logs.sh Examples/grid_map.xml Examples/demo_task 10 20 Examples/config.xml

Would create 20 log files with 10 agents with goal and start locations in Examples/grid_map.xml

After that, you can copy all of the task logs created into the VMAS/expert/logs directory so they can be used by the expert.
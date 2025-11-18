roslaunch random_map_generator test.launch            & sleep 1; 
roslaunch plan_manage plan_node.launch & sleep 1; # for benchmark
wait

#mean tracking-error, trajectory_length,  mean acc, mean ω, mean jerk noise xulong end error

#collision-number, time, replan number hzc

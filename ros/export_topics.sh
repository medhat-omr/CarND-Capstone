# setup the environment
source devel/setup.bash

# run roscore first
roscore &
# give time (in seconds) for roscore to initalize
# feel free to increase if needed
sleep 5

# for storing exported topic files
mkdir -p csv_files

# export topics from the rosbag file
# add more topics if needed
echo exporting /base_waypoints
rostopic echo -b ../udacity_succesful_light_detection.bag -p /base_waypoints >csv_files/base_waypoints.csv

echo exporting /final_waypoints
rostopic echo -b ../udacity_succesful_light_detection.bag -p /final_waypoints >csv_files/final_waypoints.csv

echo exporting /current_pose
rostopic echo -b ../udacity_succesful_light_detection.bag -p /current_pose >csv_files/current_pose.csv

echo exporting /traffic_waypoint
rostopic echo -b ../udacity_succesful_light_detection.bag -p /traffic_waypoint >csv_files/traffic_waypoint.csv

# plot the topics
# as of now, only the above topics are drawn
echo Now plotting the topics
python plot_rosbag.py

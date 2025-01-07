DIRECTORY="rosbag"

if ! command -v "roslaunch" 2>&1 >/dev/null
then
  source /opt/ros/noetic/setup.bash
fi

if [ "$#" -eq 0 ]; then
  echo "Please provide file name of ros bag file."
  exit 1
fi

if [ ! -d "$DIRECTORY" ]; then
  echo "Rosbag directory not found."
  exit 1
fi

file_name="$1"
if [[ ! "$file_name" == *.bag ]]; then
  file_name="$file_name.bag"
fi

file_path=$DIRECTORY/$file_name
if [ -d "$file_path" ]; then
  echo "$file_path does already exist."
  exit 1
fi

if [ ! -d "$file_path" ]; then
  echo "Rosbag file not found."
  exit 1
fi

roscore >/dev/null 2>&1 &
rosparam set use_sim_time true
rosbag play "$file_path" --clock
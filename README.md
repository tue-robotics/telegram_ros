# telegram_ros

Bridges between a [Telegram](https://telegram.org/) conversation and [ROS](http://ros.org).

## Installation (from source)

Go to the `src` directory of your catkin workspace and clone the package from github:

    git clone https://github.com/tue-robotics/telegram_ros.git
    
Install the required dependencies:

    rosdep install --from-path -y -i telegram_ros
    
Compile your workspace

    catkin build # or catkin_make (make sure to refresh your workspace env afterwards)

## Run

    rosrun telegram_ros telegram_ros_bridge _token:=[YOUR_BOT_API_TOKEN]

## ROS API

### Topics

#### Output

- `message_from_telegram` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
- `photo_from_telegram` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

#### Input

- `message_to_telegram` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
- `photo_to_telegram` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))

### Parameters

- `~token` (string): Telegram BOT API token
- `~caption_as_frame_id` (bool): Whether to put the caption of the image in the frame_id (default=`False`)

# telegram_ros

[![CI](https://github.com/tue-robotics/telegram_ros/actions/workflows/main.yml/badge.svg)](https://github.com/tue-robotics/telegram_ros/actions/workflows/main.yml) [![Lint](https://github.com/tue-robotics/telegram_ros/actions/workflows/lint.yml/badge.svg)](https://github.com/tue-robotics/telegram_ros/actions/workflows/lint.yml)

Bridges between a [Telegram](https://telegram.org/) conversation and [ROS](http://ros.org).

Only a single Telegram user can send and receive text, images and locations to/from the ROS bridge.
A user must send the `/start` command to start a conversation. A new user can then `/start` as well and take over.
Once connected, a user can type `/stop` to disconnect

Currently, there is *no* authentication (Issue #6)

## Installation (from source)

Go to the `src` directory of your catkin workspace and clone the package from github:

```bash
git clone https://github.com/tue-robotics/telegram_ros.git
```

Install the required dependencies:

```bash
rosdep install --from-path -y -i telegram_ros
```

Compile your workspace

```bash
catkin build # or catkin_make (make sure to refresh your workspace env afterwards)
```

## Create a bot

If you don't have a bot yet, chat to [BotFather](https://core.telegram.org/bots#6-botfather) in Telegram to create one. You can provide a name for your bot and you will receive the API token.

## Run

```bash
rosrun telegram_ros telegram_ros_bridge _token:="YourToken:FromBotFather"

rosparam set /telegram_ros_bridge/whitelist '[1, 2]'  # For the chat IDs that should be allowed
```

## ROS API

### Topics

#### Output

- `message_to_ros` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
- `image_to_ros` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
- `location_to_ros` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

#### Input

- `message_from_ros` ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))
- `image_from_ros` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
- `location_from_ros` ([sensor_msgs/NavSatFix](http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html))

### Parameters

- `~token` (string): Telegram BOT API token
- `~caption_as_frame_id` (bool): Whether to put the caption of the image in the frame_id (default: `False`)
- `~whitelist` (bool or list): False when everyone can use the bot or
    a list of telegram user IDs that are allowed to talk with the bot (default: `False`)

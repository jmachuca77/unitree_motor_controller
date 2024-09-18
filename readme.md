# Unitree Motor Controller

This package provides a ROS 2 node for controlling Unitree motors. The node subscribes to the `joint_position` topic and publishes the motor state to the `joint_states` topic.

## Installation

1. Clone the repository into your ROS 2 workspace:
    ```sh
    cd ~/bdx_ws/src
    git clone <repository_url> unitree_motor_controller
    ```

2. Build the workspace:
    ```sh
    cd ~/bdx_ws
    colcon build
    ```

3. Source the workspace:
    ```sh
    source ~/bdx_ws/install/setup.bash
    ```

## Usage

### Launching the Node

You can launch the `motor_controller` node using the provided launch file:

```sh
ros2 launch unitree_motor_controller motor_controller_launch.py
```

### Setting Parameters

Parameters can be set in the launch file or overridden from the command line. The available parameters are:

- `publish_rate`: The rate at which the motor state is published (default: 100).
- `motor_type`: The type of the motor (default: "GO_M8010_6").
- `motor_id`: The ID of the motor (default: 0).
- `joint_name`: The name of the joint (default: "unitree_motor_joint").

#### Example with Default Parameters

```sh
ros2 launch unitree_motor_controller motor_controller_launch.py
```

#### Example with Custom Parameters

```sh
ros2 launch unitree_motor_controller motor_controller_launch.py publish_rate:=50 motor_type:=A1 motor_id:=1 joint_name:=custom_joint
```

## Topics

### Subscribed Topics

- `/joint_position` (`std_msgs::msg::Float64`): The desired joint position.

### Published Topics

- `/joint_states` (`sensor_msgs::msg::JointState`): The current state of the joint.

## Testing

### Unit Tests

To run unit tests, use the following command:

```sh
colcon test --packages-select unitree_motor_controller
```

### Manual Testing

1. Launch the node:
    ```sh
    ros2 launch unitree_motor_controller motor_controller_launch.py
    ```

2. Publish a test message to the `joint_position` topic:
    ```sh
    ros2 topic pub /joint_position std_msgs/msg/Float64 "{data: 1.0}"
    ```

3. Subscribe to the `joint_states` topic to verify the output:
    ```sh
    ros2 topic echo /joint_states
    ```

## License

This project is licensed under the MIT License - see the `LICENSE` file for details.

### Licensing Information from unitree_actuator_sdk

The `unitree_motor_controller` package includes code from the `unitree_actuator_sdk` project. The `unitree_actuator_sdk` is licensed under the BSD-3-Clause License. Below is the licensing information for the `unitree_actuator_sdk`:

```text
BSD 3-Clause License

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
```
## Authors

- **Jaime Machuca** - *Initial work* - [GitHub](https://github.com/jmachuca77)

See also the list of [contributors](https://github.com/jmachuca77/unitree_motor_controller/contributors) who participated in this project.
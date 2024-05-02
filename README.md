*`Established: 2024/05/02`* *`Updated: 2024/05/02`*

## About The Project
The vehicle surrounding emergencies management for the robot vehicle ver. 1 project.

This service is used to manage the 8-direction surrounding emergencies represented by the SurroundEmergency under `vehicle_interfaces/msg_content/SurroundEmergency.msg`. Each SurroundEmergency contains the emergency score and lifetime. The service will automatically remove the expired emergency scores while clients request the emergency scores.

**NOTE:** The implementation of communication between the safety server and the client was implemented under `SafetyNode` at `vehicle_interfaces/safety.h`. The client device can easily communicate with the safety server by inheriting the `SafetyNode` class or the derived class `VehicleServiceNode` class.

**NOTE:** To enable the funcitons of `SafetyNode`, make sure to pass the correct service name to the `SafetyNode` constructor. If user want to disable the functions, set the service name to empty string.

For the client device, there are several functions to communicate with the safety server:
- `setEmergency()`: Register the emergency scores to the safety server.
- `setEmergencies()`: Register multiple emergency scores to the safety server.
- `getEmergency()`: Request the specific emergency score from the safety server.
- `getEmergencies()`: Request all the emergency scores from the safety server.



## Getting Started

### Prerequisites
- ROS2 `Foxy` or later (`Humble` recommended)
    Install ROS2 from official website: [ROS2 official website](https://docs.ros.org/en/humble/Installation.html) or simply run the following command to automatically install ROS2:
    ```bash
    curl -fsSL ftp://61.220.23.239/scripts/install-ros2.sh | bash
    ```
    **NOTE:** The script only supports `Foxy` and `Humble` versions depending on the Ubuntu version.
    **NOTE:** The script will create a new workspace at `~/ros2_ws`.
    **NOTE:** The script will create an alias `humble` or `foxy` for global ROS2 environment setup (e.g. `source /opt/ros/<$ROS_DISTRO>/setup.bash`) depending on the ROS2 version.
- [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git)

The required packages are listed in the `requirements_apt.txt` file. Install the required packages by running the following command:
```bash
xargs sudo apt install -y < requirements_apt.txt
```
**NOTE:** The required packages will be installed automatically while installing the package using the (`vcu-installer`)[https://github.com/cocobird231/RV1-vcu-install.git].


### Installation
There are two ways to install the package: manually or using the `vcu-installer`. 

#### Install Manually
1. Check if `vehicle_interfaces` package is installed. If not, install the package by following the instructions in the [vehicle_interfaces](https://github.com/cocobird231/RV1-vehicle_interfaces.git).
2. Clone the repository under `~/ros2_ws/src` and rename it to `cpp_safetyserver`:
    ```bash
    git clone https://github.com/cocobird231/RV1-safetyserver.git cpp_safetyserver
    ```
3. Change the directory to the `~/ros2_ws` workspace and build the package:
    ```bash
    # Change directory to workspace.
    cd ~/ros2_ws

    # Source the local environment.
    . install/setup.bash

    # Build the package.
    colcon build --symlink-install --packages-select cpp_safetyserver
    ```
    **NOTE:** The package is installed in the local workspace.


#### Install Using `vcu-installer`
1. Run the installer and press `Scan` button under Package Management window. If the installer not installed, install the installer by following the instructions in the [`vcu-installer`](https://github.com/cocobird231/RV1-vcu-install.git).

2. Checked the `Safety Server` checkbox under package list, right-click to modify the internet setting, then press the `Install` button to install the package.

3. The installer will create the start-up script for the package under `/etc/xdg/autostart` directory. The package will be started automatically after the system boot-up.


## Usage
The package contains two executables: `server` and `control`. The `server` executable is used to run the main service, while the `control` executable is used to control the service.

### Run the Main Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the main service:
    - Using the `launch`:
        ```bash
        ros2 launch cpp_safetyserver launch.py
        ```
        **NOTE:** The launch file parsed the `common.yaml` file to set the parameters. The `common.yaml` file is located in the `cpp_safetyserver/launch` directory.
        **NOTE:** The `common.yaml` file default the namespace to `V0`.

    - Using the `run`:
        ```bash
        ros2 run cpp_safetyserver server
        ```

### Control the Service
1. Source the local workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

2. Run the control service:
    ```bash
    ros2 run cpp_safetyserver control
    ```
    **NOTE:** If the main service is using the namespace (e.g. `V0`), the control service should use the same namespace to control the main service:
    ```bash
    ros2 run cpp_safetyserver control --ros-args -r __ns:=/V0
    ```


## Description

### Control the Service
The control executable demonstrates the control of the safety server using `SafetyReg.srv` and `SafetyReq.srv` under `vehicle_interfaces` package. 


#### `SafetyReg.srv`
The service is used to register the emergency scores (one or more) to the safety server. The service contains the following fields:
```.srv
# Request field
## Passing device_id to safety server is necessary for control server to identify the place where emergency occurred.
string[] device_id_vec

## Register emergency scores to safety server.
SurroundEmergency[] emergency_scores

# Response field
bool response # Whether the service is successfully executed.
```
The `SurroundEmergency` message stores the 8-direction emergency score and lifetime. The message contains the following fields:
```.msg
# emergency_percentages from 0.0 to 1.0 represents emergency level from low to high.
# The indices of array from 0 to 7 represents the 8-direction [F, B, L, R, FL, FR, BL, BR] respectively.
# F: forward, B: backward, L: left, R: right, FL: forward-left, FR: forward-right, BL: backward-left, BR: backward-right
# Set value < 0.0 if emergency score not used. The value is considered as 1.0 if value > 1.0
float32[8] emergency_percentages [-1, -1, -1, -1, -1, -1, -1, -1]

# lifetime_ms declares the lifetime of emergency_percentages.
float32 lifetime_ms 500.0
```
**NOTE:** The size of `device_id_vec` and `emergency_scores` should be the same.


#### `SafetyReq.srv`
The service is used to request the emergency scores from the safety server. The service contains the following fields:
```.srv
# Request field
## Get <device_id> SurroundEmergency.
## If <device_id> set to "all", return all SurroundEmergency.
## If <device_id> set to "nearest", return nearest 8-direction scores.
string device_id

# Response field
## response is true if value accepted, otherwise server ignore the request and response false.
bool response

## Return requested <device_id>. If <device_id> not found, the vector size will be 0.
string[] device_id_vec

## Get a vector of SurroundEmergency decided by request device_id. If device_id not found, the vector size will be 0.
SurroundEmergency[] emergency_scores
```
**NOTE:** If request `device_id` is set to `all`, the server will return all registered `device_id` and `emergency_scores`. If request `device_id` is set to `nearest`, the server will return the nearest 8-direction scores.

**NOTE:** If request `device_id` is invalid, the server will return the empty vector for `device_id_vec` and `emergency_scores`, and set the `response` to `false`.


### `common.yaml` File
The `common.yaml` file is currently not used. The service name will be determined by the `safetyService` under `service.json` file.

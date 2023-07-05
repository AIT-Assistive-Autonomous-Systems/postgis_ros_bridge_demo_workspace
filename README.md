<p align="center">
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge_demo_workspace/blob/humble/LICENSE">
        <img alt="GitHub" src="https://img.shields.io/github/license/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge_demo_workspace?label=Demo%20Workspace%20License">
  </a>
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge_demo_workspace/actions/workflows/ros.yaml">
        <img alt="GitHub Workflow Status (with event)" src="https://img.shields.io/github/actions/workflow/status/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge_demo_workspace/ros.yaml?label=Demo%20Build%20%26%20Test">
  </a>
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge_demo_workspace/issues">
      <img alt="GitHub issues" src="https://img.shields.io/github/issues/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge_demo_workspace?label=Demo%20Issues">
  </a>
</p>

<p align="center">
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/blob/main/LICENSE">
        <img alt="GitHub" src="https://img.shields.io/github/license/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge?label=PostGIS%20ROS2%20Bridge%20License">
  </a>
    <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/actions/workflows/linting.yaml">
        <img alt="GitHub Workflow Status (with event)" src="https://img.shields.io/github/actions/workflow/status/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/linting.yaml?label=Linting">
  </a>
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/actions/workflows/ros_humble.yaml">
        <img alt="GitHub Workflow Status (with event)" src="https://img.shields.io/github/actions/workflow/status/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/ros_humble.yaml?label=ROS2%20Humble%20Tests">
  </a>
<a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/actions/workflows/ros_iron.yaml">
        <img alt="GitHub Workflow Status (with event)" src="https://img.shields.io/github/actions/workflow/status/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/ros_iron.yaml?label=ROS2%20Iron%20Tests">
  </a>
  <a href="https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge/issues">
      <img alt="GitHub issues" src="https://img.shields.io/github/issues/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge?label=Issues">
  </a>
</p>

<p align="center">
  <a href="https://postgis.net">
        <img height="75" alt="PostGIS" src="https://upload.wikimedia.org/wikipedia/commons/7/7b/Logo_square_postgis.png">
  </a>
  <a href="https://ros.org">
        <img height="75" alt="PostGIS" src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg">
  </a>
</p>

# PostGIS ROS2 Bridge - Demo Workspace
Demo VSCode repository with all dependencies to connect to a PostgreSQL database with data stored as PostGIS elements. 

The `postgis_ros_bridge` publisher node is used to fetch data from the database and publish it as ROS2 messages.

## Setup tl;dr

* Checkout repository and open in VSCode
* Open in devcontainer
* Run tasks:
  1. `import from workspace file`
  2. `install dependencies`
  3. `build`
  4. `source ros`

For detailed info, see Section [Setup - Detailed Step-by-Step](#setup---detailed-step-by-step)

## Demo - Gravel Quarry with OpenStreetMap data integration

To start the demo (after sucessfully set up the workspace), the demo can be launched using:
````bash
cd  /workspaces/postgis_ros_bridge_demo_workspace/
source install/setup.bash
ros2 launch postgis_ros_bridge_demo gravel_quarry.launch.py
````

The launchfile starts the bridge itself, the needed transformations for visualization, a transformer node to convert gps messages to odometry (for visualization) and rviz2.

To visualize "live" data in this demo, play back the bagfile in `src/postgis_ros_bridge_demo/data/roscon_demo` via:
````bash
ros2 bag play src/postgis_ros_bridge_demo/data/roscon_demo
````

Th result should look as follows:

<table>
  <tr>
    <td align="center">Top-down view of a gravel quarry with semantic data from OSM as ROS2 marker visualized in rviz2.</td>
    <td align="center">Bird's eye view on gravel quarry with zoom to powerline visualization (ROS2 marker).</td>
    <td align="center">Same rviz2 visualization with added odometry history from rosbag (demo for "live" usecase).</td>
  </tr>
  <tr>
    <td valign="top"><img src="src/postgis_ros_bridge_demo/data/rviz_view.png"></td>
    <td valign="top"><img src="src/postgis_ros_bridge_demo/data/rviz_view_2.png"></td>
    <td valign="top"><img src="src/postgis_ros_bridge_demo/data/rviz_view_odometry.png"></td>
  </tr>
 </table>


### Demo - with foxglove

To run the demo with foxglove, use 

```bash
ros2 launch postgis_ros_bridge_demo gravel_quarry.launch.py foxglove:=true rviz:=false
```

in foxglove:
  1. Connect to `ws://localhost:8765`
  2. Load Layout (In top bar: `Layout->Import from file` ) from `/workspaces/postgis_ros_bridge_demo_workspace/src/postgis_ros_bridge_demo/rviz/gravel quarry.json`
  3. Set `ROS_PACKAGE_PATH` under `Profile->Settings` to `/workspaces/postgis_ros_bridge_demo_workspace/install/share` (needed to find meshes)

The result should look as follows: 

<img src="src/postgis_ros_bridge_demo/data/foxglove-demo.png" width="500">


## Setup - Detailed Step-by-Step
Clone this repository using
```bash
git clone https://github.com/AIT-Assistive-Autonomous-Systems/postgis_ros_bridge_demo_workspace.git
```
and open it in VSCode
```bash
cd postgis_ros_bridge_demo_workspace && code .
```

When you open it for the first time, you should see a little popup that asks you if you would like to open it in a container.  Say yes!

![template_vscode](https://user-images.githubusercontent.com/6098197/91332551-36898100-e781-11ea-9080-729964373719.png)

If you don't see the pop-up, click on the little green square in the bottom left corner, which should bring up the container dialog

![template_vscode_bottom](https://user-images.githubusercontent.com/6098197/91332638-5d47b780-e781-11ea-9fb6-4d134dbfc464.png)

In the dialog, select "Remote Containers: Reopen in container"

VSCode will build the dockerfile inside of `.devcontainer` for you.  If you open a terminal inside VSCode (Terminal->New Terminal), you should see that your username has been changed to `ros`, and the bottom left green corner should say "Dev Container"

![template_container](https://user-images.githubusercontent.com/6098197/91332895-adbf1500-e781-11ea-8afc-7a22a5340d4a.png)

To check out the repos in the src folder, run the task via `Terminal->Run Task..->import from workspace file`


After this, the folder `src` should contain a folder `postgis_ros_bridge`


Install the dependencies using `Terminal->Run Task..->install dependencies`


To build the workspace, run `Terminal->Run Task..->build` and to setup the ROS environment, run  `Terminal->Run Task..->source ros`

### Optional usage without GUI (e.g., rviz2)
By default, the devcontainer loads the `althack/ros2:humble-full` image as basis. If no GUI inside the container is needed (e.g., already setup ROS2 environment on host system, warning: incompatible message definition between different ROS distros may occur), the `althack/ros2:humble-dev` can be used to have smaller images.

#### Trouble-Shooting

A common issue within the docker trying to start a GUI-application like `rviz` is the following:
````bash
/workspaces/postgis_ros_bridge_demo_workspace$ rviz2
No protocol specified
qt.qpa.xcb: could not connect to display :2
qt.qpa.plugin: Could not load the Qt platform plugin "xcb" in "" even though it was found.
This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.

Available platform plugins are: eglfs, linuxfb, minimal, minimalegl, offscreen, vnc, xcb.

Aborted (core dumped)
````
If this happens, the connection to the display cannot be set up. 
To enable the user of the devcontainer to write to your X-Server, an option is to disable the server access control program for X [xhost](https://manpages.ubuntu.com/manpages/xenial/man1/xhost.1.html) at the host system:
````bash
sudo xhost +
````
_Warning:_ Be aware of possible security issues and side-effect depending on your setup, see [xhost manual](https://manpages.ubuntu.com/manpages/xenial/man1/xhost.1.html).


# Credits 

* "wheel loader" (https://skfb.ly/6vNp6) by Anton Sereda is licensed under Creative Commons Attribution-NonCommercial (http://creativecommons.org/licenses/by-nc/4.0/).
* "Power Transmission Line" (https://skfb.ly/oozIK) by combine_soldier is licensed under Creative Commons Attribution (http://creativecommons.org/licenses/by/4.0/).

# Funding
[DE] Die FFG ist die zentrale nationale Förderorganisation und stärkt Österreichs Innovationskraft. Dieses Projekt wird aus Mitteln der FFG gefördert. 

[EN] FFG is the central national funding organization and strengthens Austria's innovative power. This project is funded by the FFG. 

[www.ffg.at](www.ffg.at)

Projekt: [openSCHEMA](https://iktderzukunft.at/de/projekte/open-semantic-collaborative-hierarchical-environment-mapping.php)
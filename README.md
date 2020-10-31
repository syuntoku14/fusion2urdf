# fusion2urdf

I have stopped developing this repo, but any contributions are welcome.
This repo only supports Gazebo, if you are using pybullet, see: https://github.com/yanshil/Fusion2PyBullet.


## Updated!!!
* 2020/11/10: README fix
  * MacOS Installation command fixed in README
  * Date format unified in README to yyyy/dd/mm
  * Shifted Installation Upwards for better User Experience and easier to find
* 2020/01/04: Multiple updates:
  * no longer a need to run a bash script to convert stls
  * some cleanup around joint and transmission generation
  * defines a sample material tag instead of defining a material in each link
  * fusion2urdf now generates a self-contained ROS {robot_name}_description package
  * now launched by roslaunch {robot_name}_description display.launch
  * changed fusion2urdf output from urdf to xacro for more flexibility
  * separate out material, transmissions, gazebo elements to separate files
* 2018/20/10: Fixed functions to generate launch files
* 2018/25/09: Supports joint types "Rigid", "Slider" & Supports the joints' limit(for "Revolute" and "Slider"). 
* 2018/19/09: Fixed the bugs about the center of the mass and the inertia.


## Installation

Run the following command in your shell.

##### Windows (In PowerShell)

```powershell
cd <path to fusion2urdf>
Copy-Item ".\URDF_Exporter\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
```

##### macOS (In bash or zsh)

```bash
cd <path to fusion2urdf>
cp -r ./URDF_Exporter "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/"
```

## What is this script?
This is a fusion 360 script to export urdf from fusion 360 directly.

This exports:
* .urdf file of your model
* .launch and .yaml files to simulate your robot on gazebo
* .stl files of your model

### Sample 

The following test model doesn't stand upright because the z axis is not upright in default fusion 360.
Make sure z axis is upright in your fusion 360 model if you want. 

#### original model
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/industrial_robot.png" alt="industrial_robot" title="industrial_robot" width="300" height="300">

#### Gazebo simulation of exported .urdf and .launch
* center of mass
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/center_of_mass.png" alt="center_of_mass" title="center_of_mass" width="300" height="300">

* collision
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/collision.png" alt="collision" title="collision" width="300" height="300">

* inertia
<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/inertia.png" alt="inertia" title="inertia" width="300" height="300">


## Before using this script

Before using this script, make sure that your model has all the "links" as components. You have to define the links by creating corresiponding components. For example, this model(https://grabcad.com/library/spotmini-robot-1) is not supported unless you define the "base_link". 

In addition to that, you should be careful when define your joints. The **parent links** should be set as **Component2** when you define the joint, not as Component1. For example, if you define the "base_link" as Component1 when you define the joints, an error saying "KeyError: base_link__1" will show up.

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/spot_mini.PNG" alt="spot_mini" title="spot_mini" width="300" height="300">

Also, make sure components of your model has only bodies. **Nested components are not supported**.
For example, this works:

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/only_bodies.PNG" alt="only_bodies" title="only_bodies" width="300" height="300">

but this doesn't work since the "face (3):1" component contains other components. A component must contain only bodies:

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/nest_components.PNG" alt="nest_components" title="nest_components" width="300" height="300">

Sometimes this script exports abnormal urdf without any error messages. In that case, the joints should have problems. Redefine the joints and run again.

In addition to that, make sure that this script currently supports only "Rigid", "Slider" and "Revolute".


## How to use

As an example, I'll export a urdf file from this cool fusion360 robot-arm model(https://grabcad.com/library/industrial-robot-10).
This was created by [sanket patil](https://grabcad.com/sanket.patil-16)

### Install in Shell 

Run the [installation command](#installation) in your shell.

### Run in Fusion 360

Click ADD-INS in fusion 360, then choose ****fusion2urdf****. 

**This script will change your model. So before run it, copy your model to backup.**

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/copy.png" alt="copy" title="copy" width="300" height="300">

Run the script and wait a few seconds(or a few minutes). Then a folder dialog will show up. Choose where you want to save the urdf (A folder "Desktop/test" is chosen in this example").
Maybe some error will occur when you run the script. Fix them according to the instruction. In this case, something wrong with joint "Rev 7". Probably it can be solved by just redefining the joint.

![error](https://github.com/syuntoku14/fusion2urdf/blob/images/error.png)

**You must define the base component**. Rename the base component as "base_link". 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/cautions.PNG" alt="cautions" title="cautions" width="300" height="300">

In the above image, base_link is gounded. Right click it and click "Unground". 

Now you can run the script. Let's run the script. Choose the folder to save and wait a few second. You will see many "old_component" the components field but please ignore them. 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/result.PNG" alt="results" title="results" width="250" height="300">

You have successfully exported the urdf file. Also, you got stl files in "Desktop/test/mm_stl" repository. This will be required at the next step. The existing fusion CAD file is no more needed. You can delete it. 

The folder "Desktop/test" will be required in the next step. Move them into your ros environment.


#### In your ROS environment

Place the generated _description package directory in your own ROS workspace. "catkin_ws" is used in this example.
Then, run catkin_make in catkin_ws.

```bash
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

Now you can see your robot in rviz. You can see it by the following command.

```bash
roslaunch (whatever your robot_name is)_description display.launch
```

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/rviz_robot.png" alt="rviz" title="rviz" width="300" height="300">

If you want to simulate your robot on gazebo, just run
```bash
roslaunch (whatever your robot_name is)_description gazebo.launch
```

**Enjoy your Fusion 360 and ROS life!**

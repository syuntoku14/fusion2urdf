# fusion2urdf

### Updated!!!

* 2018/09/19: Fixed the bugs about the center of the mass and the inertia. 
* 2018/09/25: Supports joint types "Rigid", "Slider" & Supports the joints' limit(for "Revolute" and "Slider"). 
* 2018/10/20: Fixed fucnctions to generate launch files

## What is this script?
This is a fusion 360 script to export urdf from fusion 360 directly.

This exportes:
* .urdf file of your model
* .launch and .yaml files to simulate your robot on gazebo
* .stl files of your model

**convertSTL.rb** was created by [@Chris Polis](https://github.com/cmpolis/convertSTL#author)

### Sample 

The following test model doesn't stand upright because z axis is not upright in default fusion 360.
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

Before using this script, make sure that your model has all the "links" as componens. You have to define the links by creating corresiponding components. For example, this model(https://grabcad.com/library/spotmini-robot-1) is not supported unless you define the "base_link". 

In addition to that, you should be carefult when define your joints. The **parent links** should be set as **Component2** when you define the joint, not as Component1. For example, if you define the "base_link" as Component1 when you define the joints, an error saying "KeyError: base_link__1" will show up.

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/spot_mini.PNG" alt="spot_mini" title="spot_mini" width="300" height="300">

Also, make sure components of your model has only bodies. **Nested components are not supported**.
For example, this works:

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/only_bodies.PNG" alt="only_bodies" title="only_bodies" width="300" height="300">

but this doesn't work since the "face (3):1" component contains other components. A component must contain only bodies:

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/nest_components.PNG" alt="nest_components" title="nest_components" width="300" height="300">

Sometimes this script exports abnormal urdf without any error messages. In that case, the joints should have problems. Redefine the joints and run again.

In addition to that, make sure that this script currently supports only "Rigid", "Slider" and "Revolute". I will add new joint types someday.

## How to use

As an example, I'll export a urdf file from this cool fusion360 robot-arm model(https://grabcad.com/library/industrial-robot-10).
This was created by [sanket patil](https://grabcad.com/sanket.patil-16)

#### In Fusion 360
##### Install

Run the following command in your shell.

###### Windows (In PowerShell)

```powershell
cd <path to fusion2urdf>
Copy-Item ".\URDF_Exporter\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
```

###### macOS (In Bash)

```bash
cd <path to fusion2urdf>
cp -r ../URDF_Exporter "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts/"
```

##### Run

Click ADD-INS in fusion 360, then choose URDF_Exporter. 

**This script will change your model. So before run it, copy your model to backup.**

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/copy.png" alt="copy" title="copy" width="300" height="300">

Run the script and wait a few seconds(or a few minutes). Then a folder dialog will show up. Choose where you want to save the urdf(A folder "Desktop/test" is chosen in this example").
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

Place this repository at your own ROS workspace. "catkin_ws" is used in this example.
Then, run catkin_make in catkin_ws.

```bash
cd ~/catkin_ws/src
git clone git@github.com:syuntoku14/fusion2urdf.git
cd ..
catkin_make
source devel/setup.bash
```

Next, copy the repository named your robot's name that you made in the previous step and paste it at "~/catkin_ws/src/fusion2urdf". In this example, I copied "industrial_robot" which is located in the "test" folder.  

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/fusion2urdf.png" alt="folder" title="folder" width="300" height="150">

Then, run the "stl2binary.bash" with your robot's name. You need to install Ruby because there is a ruby code "convertSTL.rb". 

```bash
cd ~/catkin_ws/src/fusion2urdf
bash stl2binary.bash industrial_robot
```

The bin_stl in your robot's name folder contains binary stl files for urdf \<mesh\> tags.

Now you can see your robot in rviz. You can see it by the following command.

```bash
cd ~/catkin_ws/src/fusion2urdf/industrial_robot
roslaunch urdf_tutorial display.launch model:=industrial_robot.urdf
```

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/rviz_robot.png" alt="rviz" title="rviz" width="300" height="300">

If you want to simulate your robot on gazebo, place exported .launch and .yaml files at ~/catkin_ws/src/fusion2urdf/launch.
Then run "roslaunch industrial_robot.launch".

```bash
mkdir ~/catkin_ws/src/fusion2urdf/launch

cp ~/catkin_ws/src/fusion2urdf/industrial_robot/industrial_robot.launch ~/catkin_ws/src/fusion2urdf/industrial_robot/industrial_robot_controller.yaml ~/catkin_ws/src/fusion2urdf/launch/

roslaunch ~/catkin_ws/src/fusion2urdf/launch/industrial_robot
```

**Enjoy your Fusion 360 and ROS life!**

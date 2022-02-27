# fusion2urdf

I have stopped developing this repo, but any contributions are welcome.
This repo only supports Gazebo, if you are using pybullet, see: https://github.com/yanshil/Fusion2PyBullet.


## Updated!!!
* 2021/01/09: Fix xyz calculation. 
  * If you see that your components move arround the map center in rviz try this update 
  * More Infos see: https://forums.autodesk.com/t5/fusion-360-api-and-scripts/difference-of-geometryororiginone-and-geometryororiginonetwo/m-p/9837767

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


## Complex Kinematic Loops and Spherical joints (may be fixed later):

DO NOT use Fusion 360's inbuilt joint editor dialouge for positioning joints

For example, [@rohit-kumar-j](https://github.com/rohit-kumar-j) had this complicated robot to assemble. There are over.. some 50 joints in all, including some forming loops within the structure like a [4-bar mechanism](https://www.youtube.com/watch?v=eYOt6SEKHFs&ab_channel=YuhangHu), also called **kinematic loops**.

![image](https://user-images.githubusercontent.com/37873142/133144979-30218496-09d4-40bb-9af7-95448a7665ee.png)

As you can see below, when fusion initailly forms joints, it might not align where you want it to align to. In the image below, the cylinder's cap side doesn't exaclty coincide with the position of the pin where it needs to be join. The red arrow shows the mismatch in initial joint positioning by fusion.

![image](https://user-images.githubusercontent.com/37873142/133145309-298f17a4-bd62-48fa-b1c2-54f58e26fce4.png)

If you were to manually drag the parts and align them as shown below, it would cause cascading problems with the visual and collision properties of certain links. 

![Capture](https://user-images.githubusercontent.com/37873142/133146628-c4c2b8dd-ac7b-41e8-bd62-1d2c2b80adce.PNG)

Below you can see one of the cylinders is mismatched as compared to the others (red and grey colors are cylinders) 
(The below urdf is visualized in pybullet)

![image](https://user-images.githubusercontent.com/37873142/133141659-440a0a4a-1afa-4751-99ba-fc3db02f7450.png)

See Also: Similar to this issue, but only for a few axes [here](https://github.com/yanshil/Fusion2PyBullet/issues/6) (turns out there was a fusion API change back then, and the exporter wasn't yet updated [See this commit](https://github.com/syuntoku14/fusion2urdf/commit/8786e6318cdcaaf32070148451a27ab6e4f6697d), but it now is)


**The fix for this is to leave Fusion's joint controls unedited and form joints for the robot joints (See below)**


A similar issue with another set of joints at the ankle was fixed by following the above fromat. [Here is the video](https://www.youtube.com/watch?v=0hfkm7vv5o8&ab_channel=JRohit)

For spherical joints, it is better to keep them revolute and define the joints as spherical, later in the generated URDF(provided the urdf parser in your visualizer/physics engine(gazebo,webots,pybullet,mujoco,etc) supports spherical joints, in pybullet it does).
The ankle joint below has 4 spherical joints and only two of them were defined as revolute while exporting from fusion 360. The other 2 spherical joints were created in pybullet using pybullet's inbuilt functions for creating kinematic loops.(see the gif below)

![youtube-video-gif](https://user-images.githubusercontent.com/37873142/133144404-45d9e444-8ddb-4b5f-8970-6e637b750faa.gif)


## In some cases, before export Turn off "Capture design history"

For preplanning the component placement when working/assembling your own robot. It is recomended to have separate names for components and save individual components in a separate folder, create a back up and, break link with the original. This folder can be later deleted after genearating the urdf. See [Issue #51](https://github.com/syuntoku14/fusion2urdf/issues/51) for problem with "copy-paste" vs "copy-paste new".



## How to use

As an example, I'll export a urdf file from this cool fusion360 robot-arm model(https://grabcad.com/library/industrial-robot-10).
This was created by [sanket patil](https://grabcad.com/sanket.patil-16)

### Install in Shell 

Run the [installation command](#installation) in your shell.

### Run in Fusion 360

Click ADD-INS in fusion 360, then choose ****fusion2urdf****. 

**This script will change your model. So before running it, copy your model to backup.**

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/copy.png" alt="copy" title="copy" width="300" height="300">

Run the script and wait a few seconds(or a few minutes). Then a folder dialog will show up. Choose where you want to save the urdf (A folder "Desktop/test" is chosen in this example").
Maybe some error will occur when you run the script. Fix them according to the instruction. In this case, something wrong with joint "Rev 7". Probably it can be solved by just redefining the joint.

![error](https://github.com/syuntoku14/fusion2urdf/blob/images/error.png)

**You must define the base component**. Rename the base component as "base_link". 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/cautions.PNG" alt="cautions" title="cautions" width="300" height="300">

In the above image, base_link is grounded. Right-click it and click "Unground". 

Now you can run the script. Let's run the script. Choose the folder to save and wait for a few seconds. You will see many "old_components" in the components field, please ignore them. 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/result.PNG" alt="results" title="results" width="250" height="300">

You have successfully exported the urdf file. Also, you got `.stl` files in the "Desktop/test/mm_stl" repository. This will be required at the next step. The existing fusion CAD file is no more needed. You can delete it. 

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



# Citation

```
@misc{toshinori2020fusion2urdf,
    author = {Toshinori Kitamura},
    title = {Fusion2URDF},
    year = {2020},
    publisher = {GitHub},
    journal = {GitHub repository},
    howpublished = {\url{https://github.com/syuntoku14/fusion2urdf}}
}
```

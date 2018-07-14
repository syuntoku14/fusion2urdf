# fusion2urdf

This is a fusion 360 script to export urdf from fusion 360 directly.

Currenty, this script supports only revolute joints. Let me know if your model has joints except for revolute type.

**convertSTL.rb** was created by [@Chris Polis](https://github.com/cmpolis/convertSTL#author)

## Before using this script

Before using this script, make sure that your model consists of only components. Also, your components must have only one occurence. For example, the following model(https://grabcad.com/library/spotmini-robot-1) is not supported unless you fix them. The base_link doesn't exist and some components have two occurences(For instance, Component1 has two occurences Commponent1:1 and Component1:2). I'll fix these issues someday.

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/spot_mini.PNG" alt="spot_mini" title="spot_mini" width="300" height="300">


## How to use

As an example, I'll export a urdf file from this cool fusion360 robot-arm model(https://grabcad.com/library/industrial-robot-10).
This was created by [sanket patil](https://grabcad.com/sanket.patil-16)

#### In Fusion 360

At first, we have to install this script. Download this repository and unzip at anywhere you like. The "URDF_Exporter" in this repository is the folder containing the fusion2urdf script.

Copy and paste URDF_Exporter at the repository where your fusion scripts locate. In windows, it is usually at 

```bash
'C:\Users\username\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts'
```

Then, click ADD-INS in fusion 360. Click the green plus button and choose URDF_Exporter. 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/install_script.PNG" alt="install" title="install" width="300" height="300">

This script will change your model. So before run it, copy your model to backup.

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/copy.png" alt="copy" title="copy" width="300" height="300">

Run the script. Then a folder dialog will show up. Choose where you want to save the urdf(A folder "Desktop/test" is chosen in this example").
Maybe some error will occur when you run the script. Fix them according to the instruction. In this case, something wrong with joint "Rev 7". Probably it can be solved by just redefining the joint.

![error](https://github.com/syuntoku14/fusion2urdf/blob/images/error.png)

The name of components can't contain any spaces. Also, you must define the base component. Rename the base component as "base_link". 

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

Then, run the "stl2binary.bash" with your robot's name. 

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

**Enjoy your Fusion 360 and ROS life!**

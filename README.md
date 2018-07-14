# fusion2urdf

This is a fusion 360 script to export urdf from fusion 360 directly.

## How to use

As an example, I'll export a urdf file from this cool fusion360 robot-arm model(https://grabcad.com/library/industrial-robot-10).
This was created by [sanket patil](https://grabcad.com/sanket.patil-16)

#### At Fusion 360

At first, we have to install this script. Download this repository and unzip at anywhere you like. The "URDF_Exporter" in this repository is the folder containing the fusion2urdf script.

Copy and paste URDF_Exporter at the repository where your fusion scripts locate. In windows, it is usually at 

```bash
'C:\Users\username\AppData\Roaming\Autodesk\Autodesk Fusion 360\API\Scripts'
```

Then, click ADD-INS in fusion 360. Click the green plus button and choose URDF_Exporter. 

![install](https://github.com/syuntoku14/fusion2urdf/blob/images/install_script.PNG =200x200)

This script change your model. So before run it, copy your model to backup.

![copy](https://github.com/syuntoku14/fusion2urdf/blob/images/copy.png =200x200)

Run the script. Then a folder dialog will show up. Choose where you want to save the urdf.
Maybe some error will occur when you run the script. Fix them according to the instruction. In this case, something wrong with joint "Rev 7". Probably it can be solved just redefine the joint.

![error](https://github.com/syuntoku14/fusion2urdf/blob/images/error.png)

The name of components con't contain any spaces. Also, you must define the base component. Rename the base component as "base_link".



#### At your ROS environment

Place this repository at your own ROS workspace. "catkin_ws" is used in this example.
Then, run catkin_make in catkin_make.

```bash
cd ~/catkin_ws/src
git clone git@github.com:syuntoku14/fusion2urdf.git
cd ..
catkin_make
```

**convertSTL.rb** was created by [@Chris Polis](https://github.com/cmpolis/convertSTL#author)

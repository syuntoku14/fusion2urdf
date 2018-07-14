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

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/install_script.PNG" alt="install" title="install" width="300" height="300">

This script change your model. So before run it, copy your model to backup.

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/copy.png" alt="copy" title="copy" width="300" height="300">

Run the script. Then a folder dialog will show up. Choose where you want to save the urdf(A folder "Desktop/test" is chosen in this example").
Maybe some error will occur when you run the script. Fix them according to the instruction. In this case, something wrong with joint "Rev 7". Probably it can be solved just redefine the joint.

![error](https://github.com/syuntoku14/fusion2urdf/blob/images/error.png)

The name of components con't contain any spaces. Also, you must define the base component. Rename the base component as "base_link". 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/cautions.PNG" alt="cautions" title="cautions" width="300" height="300">

In the above image, base_link is gounded. Right click it and click "Unground". 

Now you can run the script. Let's run the script. Choose the folder to save and wait a few second. You will see many "old_component" the components field but please ignore them. 

<img src="https://github.com/syuntoku14/fusion2urdf/blob/images/result.PNG" alt="results" title="results" width="300" height="300">

You have successfully exported the urdf file. Also, you got stl files in "Desktop/test/mm_stl" repository. This will be required at the next step. The existing fusion CAD file is no more needed. You can delete it. 

The folder "Desktop/test" will be required in the next step. Move them into your ros environment.


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

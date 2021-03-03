# Prerequisites
## General
- Install git-lfs: https://git-lfs.github.com/
- Install ROS Melodic on 18.04
- sudo apt-get install python-catkin-tools
- Call 'software-properties-gtk' in a terminal 
and enable the 'source code' checkbox in the opened window. 
Do an 'apt update' or update your package sources if you are asked in that window to do so.
- sudo apt build-dep caffe-cpu # Get build dependencies
- sudo apt-get install libatlas-base-dev # Another dependency for caffe
- Install Caffee (for object recognition) by this tutorial https://caffe.berkeleyvision.org/install_apt.html :
```
sudo apt-get install libprotobuf-dev libleveldb-dev libsnappy-dev libopencv-dev libhdf5-serial-dev protobuf-compiler
sudo apt-get install --no-install-recommends libboost-all-dev
sudo apt-get install libatlas-base-dev
sudo sudo apt-get  install libgflags-dev libgoogle-glog-dev liblmdb-dev
cd ~
mkdir ~/cafferepo
cd ~/cafferepo
git clone https://github.com/BVLC/caffe
cd caffe
mkdir build
cd build
cmake .. -DCPU_ONLY=ON # you can also install the cuda related stuff and omit this, but this is not in the scope of this tutorial
make all
make install
make runtest
cd ~/cafferepo/caffe
protoc src/caffe/proto/caffe.proto --cpp_out=.
mkdir include/caffe/proto
mv src/caffe/proto/caffe.pb.h include/caffe/proto
python scripts/download_model_binary.py models/bvlc_reference_caffenet/
```
Please note: If you have an existing RS in your catkin workspace it might make sense to get the newest robosherlock and fully recompile it so caffe can be found and the CaffeAnnotator is built.

## UE4
- Install UE4.22.3
- clone RobCog fork repo: https://github.com/Sanic/RobCoG/ and checkout the 4.22bs branch
- git submodule update --init --recursive # or make sure to 'git submodule update --init' all the modules
- Open RobCog, make a copy of the IAIKitchen map to work with.
  Note: If you get a compile error that the LogROS log category can't be linked, simply exchange it with the LogTemp category in the corresponding code.
- Go to Window -> Levels and remove the IAIKitchen Items and the Hands Basic Sublevel.
- Add a new DefaultPawn and set Auto Possess Player to "Player 0"
- Add a RGBDCamera Actor to the scene and set the FOV to 62 in the "Camera Settings" and in the "RGB-D Settings".
- Also set the following settings on the RGBDCamera:
  - Width: 640
  - Height: 480
  - FrameRate: 2
  - Bind to Any IP: True
  - Capture X image: All to true
  - Color All Objects on Every Tick: True
  - Color Generation Maxmimum Amount: 10000

- Add the following tag to the RGBDCamera actor (to be more specific, click on the RGBDCamera and look in the Details pane for the 'Actor' Category which starts with Menu entries like 'Convert Actor', 'Can be damaged', etc. Expand this category in the bottom for more options to appear. There, you should be able to add the following tag to the Actor).
```SemLog;id,urobovision_camera;```
(needed as reference for pose updates with ROSWorldControl)
- Go to the C++ classes in UE4 and drag&drop one ROSWorldControlActor into your world. Select it and go to the detail pane below the world outlier. There, you can find 'ROSWorld Control' -> 'RWCRegistrationClass'. This is set to None by default. Click on it and set it to 'RWCManager'. If RWCManager doesn't show up, you might not have properly installed the UROSWorldControl plugin.
- Add an 'Object Pose Publisher' to the World. This Actor comes from https://github.com/code-iai/UnrealInterfaceObjectPlugin which is a submodule of the cloned RobCog.
- In the UE4Editor, open the Editor Preferences and search for 'cpu' to disable the 'use less cpu when running in background' flag. Otherwise the performance will drop immensly when the window doesn't have focus.
- Setup game instance in UE4 according to UROSBridge documentation: https://github.com/robcog-iai/UROSBridge/blob/master/Documentation/HandlerInGameInstance.md
- If your color image in URoboVision is white, fix it by adjusting the PostProcessVolume. In the world outlier, go to SL_IAIKitchen_SunnyLightsOff->Volumes->PostProcessVolume. Go to the "Post Process Volume Settings" -> "Blend Weight" and set it to 0.0.

## RoboSherlock and friends
- Open a terminal and cd to your catkin workspace / src
- git clone https://github.com/RobotWebTools/rosbridge_suite.git
- cd rosbridge_suite
- git checkout 0.11.3
  Note: This is currently necessary because the websocket communication doesn't work with the chosen UROSBridge and the newest rosbridge for me
- cd ..
- rosdep install rosbridge_suite (maybe prefix sudo)
- rosdep install rosbridge_server
- roscd rosbridge_server
- pip install . # maybe you have to install pip first : sudo apt install python-pip
- Start rosbridge to verify that it worked: roslaunch rosbridge_server rosbridge_websocket.launch
- cd to your catkin workspace / src
```
git clone https://github.com/sanic/rs_bs
git clone https://github.com/robosherlock/robosherlock # RS should be newer than Mar 18, 2020 or https://github.com/RoboSherlock/robosherlock/commit/80b2b1dd7a670fbc10da2cd1ecb964c234fc1d90
git clone https://github.com/bbferka/rs_addons
cd rs_addons
git checkout 829a4c23cf55eb6341c9770c8ecf4b780603b321
# Go to your CMakeLists.txt in rs_addons. Find the line find_package(RapidJSON REQUIRED) and delete it. Go to the find_package call to catkin and include rapidjson_ros there.
# Example diff:
# @@ -18,7 +18,6 @@ endif(json_prolog_FOUND)
#  find_package(Caffe QUIET)
#  find_package(aruco QUIET)
#  find_package(MPI QUIET)
# -find_package(RapidJSON REQUIRED)  # <------ !!!
#  
#  if(Caffe_FOUND)
#      add_definitions(-DWITH_CAFFE)
# @@ -31,6 +30,7 @@ endif(Caffe_FOUND)
#  find_package(catkin REQUIRED robosherlock
#      message_generation
#      message_runtime
# +    rapidjson_ros  # <------ !!!
#      ${OPTIONAL_simtrack}
#      ${OPTIONAL_json_prolog}
#      )
#
# You might also have to remove an old line from the all_types.xml file in rs_addons.
# cd to rs_addons/descriptors/typesystem/
# open all_types.xml and remove the line that includes 'addons.xml'

cd ..
git clone https://github.com/sanic/rs_resources/ # contains the newer object recognition model. Warning: git lfs must be installed
git clone https://github.com/robcog-iai/unreal_ros_pkgs.git
git clone git@github.com:code-iai/unreal_interface.git
```
- cd ~/cafferepo/caffe/models/bvlc_reference_caffenet/
- cp bvlc_reference_caffenet.caffemodel YOUR_CATKIN_WORKSPACE/src/rs_resources/caffe/models/bvlc_reference_caffenet/
- cd to YOUR_CATKIN_WORKSPACE/src/robosherlock . Don't roscd, because then you will end up in the package. You need to be in the robosherlock meta package for the next step.
- rosdep install --from-path . --ignore-src
- git submodule update --init
- cd to your catkin workspace
- catkin build
- source ~/.bashrc

# How to use
- In one terminal: roscore # or set your ROS Master to the host running ROS
- In another terminal: roslaunch rosbridge_server rosbridge_websocket.launch
- Start UE4 and load the RobCog Project.
- Use either a) live robot data or b) record&playback mongo data for development.
- For a): `rosrun robosherlock runAAE _ae:=demo_w_ue4bs,ue4bs _vis:=true`
  The mentioned AE files are located in rs_bs and are loading data from a kinect bag or live kinect
- For b): load the data into Mongo DB first by executing (WARNING: This will delete existing RS Mongo Data!):
```rosrun robosherlock run _ae:=storage```
While this program is running, open another terminal and playback the input rosbag file with:
```rosbag play YOURBAGFILE.bag```
After loading the data into the mongo database, you can stop the run _ae:=storage command. After that, start RS with:
```rosrun robosherlock runAAE _ae:=mongo_w_ue4bs,ue4bs _vis:=true```

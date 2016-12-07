# eusurdf

This package contains urdf models and moveit scene file, which are converted from https://github.com/euslisp/EusLisp/tree/master/models.
These models are automatically converted by `rosrun eusurdf convert-all-eusmodel.py` and `rosrun eusurdf convert-all-eusscene.py`.

## convert eus model to urdf

Convert irteus object/scene to urdf.

```
roscd eusurdf/euslisp
roseus convert-eus-to-urdf.l
;; convert irteus object to urdf
;; the urdf model is generated in `rospack find eusurdf`/models
(progn (load "models/laundry-machine-object.l") (irteus2urdf-for-gazebo (laundry-machine) :name "laundry"))
;; convert irteus scene to urdf and world file
;; some key arguments need to be set
;; using (generate-room-models) is recommended to convert room model
(progn (load "models/room73b2-scene.l") (irteus2urdf-room-for-gazebo (room73b2)))
;; util wrapper function to convert room model
(generate-room-models "room73b2")
```

## Spawn scene model to gazebo

- Use world file

``` bash
roslaunch gazebo_ros empty_world.launch world_name:=`rospack find eusurdf`/worlds/room73b2.world
```

- Use urdf.xacro file

``` bash
roslaunch gazebo_ros empty_world.launch
roslaunch eusurdf gazebo_spawn_room73b2.launch
```

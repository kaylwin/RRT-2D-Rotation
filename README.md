# RRT-2D-Rotation
This code is a sample implementation of a RRT algorithm with an Octree for nearest neighbor search. The 3 dimensions are 2D translation and 1D rotation. The display is written with Qt and the collison detection is written with box2d. The algorithm works as follows, a hardcoded set of obstacles are created, a hard coded start point is chosen, and goal is hard coded as well. System starts up and searches from the start point toward the goal following the RRT algorithm and avoiding obstacles. For the RRT algorithm the nearest point is chosen using an Octree lookup. The main code is in ui/Helper.cpp --> this calls out to rrt_2d/Octree.h and src/Utils.cpp. The rest is just making QT cooperate. 




## Sample execution
![sample execution](https://github.com/kaylwin/RRT-2D-Rotation/blob/main/assets/output.gif)
## Building

Depends on [box2d](https://github.com/erincatto/box2d) Please make install this using provided instructions so that CMake can find it.
```
cd rrt-2d-rotation
mkdir build
cd build
cmake ..
make 
./rrt_2d

```
## Octree
The octree is written with the X and Y axis representing translation with vertical axis representing rotation. The top and bottom of the octree is identified (meaning going down comes up through the top and vice versa) to reprent rotations that cross from 0->2PI. The distance metric for search is a Euclidean distance on x, y, theta. The logic for how that work is as follows, let the robot be represented by the rectangle in the gif above. Choose a point at the top of the rectangle center about its horizontal axis one meter forward. One radian of rotation corresponds to 1m in distance traveled about the circumference of center of the robot. Add this distance to the distance traveled by the center of the rectangle and you have a crude distance metric. This comes with NO WARRANTY either stated or implied

## Path Cleanup
Once a path is found it is cleaned up by iterating back through previous points and seeing if points can be ommitted without creating a collision

## Gridmap
Old code, basically a very inefficient search that expands outwards from a cube like an onion. I wasn't sure if this would work, it does, but not nearly as well as the octree so I threw this out.

## License
This code is meant for show -- do not use. There is no warranty either stated or implied. 

## Deps
* https://box2d.org/

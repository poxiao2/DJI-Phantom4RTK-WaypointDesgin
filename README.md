# DJI-Phantom4RTK-WaypointDesgin
desgin waypoints for phantom4RTK UAV (为精灵4RTK无人机设计航点)

## Requirement
python 3.x (the python version I used is 3.5 or higher)<br>
numpy<br>
matplotlib

## Usage
**you'd better to run the "computeWayPointsCoords.py" in the commandline:**
```
cd root_path (where you save the "computeWayPointsCoords.py" and other scripts)
python computeWayPointsCoords.py shootDistance homeAltitude waypointsNumber
```
you can find that there are three args after the script. the first arg "shootDistance" represent the distance between aircraft and target, the second arg "homeAltitude" represent the absolute altitude (WGS-84) of take off point (which is also called "home point") , the third arg "waypointsNumber" represent the number of waypoints.

currently, I just design the waypoints for the single independent target, such as pier, single building, single tree etc and so on. the rendering just like:<br>
![rendering image](https://github.com/poxiao2/image-store/blob/master/1596858826(1).png)


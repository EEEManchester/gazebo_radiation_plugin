# Gazebosim Radiation Sensor/Sources Plugin

The repository contains code to adding custom radiation sensors and sources into gazebo. 

These sensors and sources provide plugin libraries defines in the radiation_sensor/source.xml files which can be included like any other sensor into an sdf using the "plugin" tags inside the "sensor" tags.

Once loaded the sensors and sources will discover each other within gazebo and calculate the expected measurements at the sensor based on intensity dropoff (1/r^2), attenuation through objects and collomation of the sensor, based on either a sensitivity function or angular window. 

The values calculated by the sensor along with the sensor pose are then published on both gazebo topics and to rostopics,with the name taking the form of: "/radiation_sensor_plugin/sensor_X" where X is the name given to the sensor.

Info about the sources are also published on: "/radiation_sensor_plugin/source_Y/\*type\*", but this is just for debugging/sanity checking and is not used by the sensor at all.

-------

## Important note!!!

This repository borrows a script called custom sensor preloader from: https://github.com/peci1/gazebo_custom_sensor_preloader.

I should just make this depend on that package but I get compile issues when I try to do this.(Most likely user error).   

**I am not trying to take credit for peci1's god like coding of the preloader!**

-------

## Install Requirements

To use the radiaiton sources and sensors gazebosim must be of a version  7.4.* or higher. The easiest way to update this for a user running ros kinetic is to run:

**Note you may need sudo to run these commands so copy and paste one at a time and act accordingly to the responce!**

```
apt remove ros-kinetic-gazebo* gazebo* libgazebo*
```
 
Once removed we can then add the osrf gazebo repositories following these instructions: http://gazebosim.org/tutorials?tut=install_ubuntu 
or the tldr is to run:
```
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' 

cat /etc/apt/sources.list.d/gazebo-stable.list   
```

Which should return: ```deb http://packages.osrfoundation.org/gazebo/ubuntu-stable bionic main```

And then run: 

```
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

apt update

apt install ros-kinetic-gazebo* gazebo* libgazebo*

gazebo --version
```
If everything has gone correctly you should see a version number of 7.16.0 or higher.

Once updated we should be able to catkin_make the repository without any issues.

## Code

This repository contains several bits of code:

* CustomSensorPreloader.cpp/h for loading custom sensors into gazebo 
* RadiationSensor.cpp/h creates a gazebo radiation sensor 
* RadiationSource.cpp/h creates a gazebo radiation source 
* RadiationSensorPlugin.cpp/h creates a plugin interface between ros and gazebo for the sensors
* RadiationSourcePlugin.cpp/h creates a plugin interface between ros and gazebo for the sources
* gazebo.in runs gazebo with the customsensorpreloader script
* gzserver.in runs gzserver with the customsensor script
* generate_radiation_sources.py creates example radiation sources from rosparameters
* load_radiation_sources.py with add each of the sources in the sources folder into gazebo.
* model_mover.py for testing used to rotate the model called sensor in a circle

**ADD FULL CODE EXPLAINATION**


### Additional Settings

The radiation sensor has some additional settings which can be set on the param server these are type,range and attenuation_factors.

For each sensor type and range should be loaded using /SENSOR_NAME/type and /SENSOR_NAME/range where SENSOR_NAME is the sensor name e.g. sensor_0. These parameters allow you to set maximum sensing ranges and radiation types the sensor is detecting. e.g. Alpha, Beta, Gamma. If these are set the sensor will disregard anything too far away or of the wrong radiation type. 

Attenuation_factors is set using a yaml file and is explained in the running the example section.

### Running the example

The first thing which needs to be done is load some parameters into the rosparam server which relate to the radiation sources being used. Basic sources can be described in the format shown in configs/sources.yaml requiring x,y,z,type and value,noise and units. Use:
 ```
roscore

rosparam load sources.yaml
```
to load these onto the rosparam server.

We can then use:
```
rosrun gazebo_radiation_plugins generate_radiation_sources.py
```
 to create sources with the provided parameters which will be written into the sources folder. 

Optionally we can load attenuation values into the parameter server too using the configs/attenuation.yaml file.
This allows for objects to have attenuation factors different from 0 or 1(free space and blockages). 
Use: 
 ```
rosparam load attentuation.yaml
```

to load these parameters. 

Next we will bring up gazebo with the custom sensor preloader. This is done by calling 

```
rosrun gazebo_radiation_plugins gazebo --verbose      <- verbose not required just useful
```

Once running we can load our example sources using:

```
rosrun gazebo_radiation_plugins load_radiation_sources.py
```

And then finally we can load an example sensor using:
```
rosrun gazebo_ros spawn_model -file src/gazebo_radiation_plugins/templates/radiation_sensor.sdf  -sdf -x 0 -y 0 -z 0.1 -model sensor
```
(for real use this sensor would be attached to a robot).

a script to move the example sensor is included.
```
rosrun gazebo_radiation_plugins model_mover.py
```

Now you should be able to see the rostopic showing up for the sensor and how the value changes as you move it around. NB: if collomation is being use then the green square represents the front of the sensor and thus the sensor will need rotating around to face the sources. 


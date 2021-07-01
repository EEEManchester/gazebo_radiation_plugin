# Gazebosim Radiation Sensor/Sources Plugin

The repository contains code to adding custom radiation sensors and sources into gazebo. 

These sensors and sources provide plugin libraries defines in the radiation_sensor/source.xml files which can be included like any other sensor into an sdf using the "plugin" tags inside the "sensor" tags.

Once loaded the sensors and sources will discover each other within gazebo and calculate the expected measurements at the sensor based on intensity dropoff (1/r^2), attenuation through objects and collomation of the sensor, based on either a sensitivity function or angular window. 

The values calculated by the sensor along with the sensor pose are then published on both gazebo topics and to rostopics,with the name taking the form of: "/radiation_sensor_plugin/sensor_X" where X is the name given to the sensor.

Info about the sources are also published on: "/radiation_sensor_plugin/source_Y/\*type\*", but this is just for debugging/sanity checking and is not used by the sensor at all.

#### Recent Additions

* Launch files added to allow for mapping of an environement using a husky with a radiation sensor using the radiation_mapping.launch file. (requires radiation_layer for visualisation)
* More environments added, including some (reactor room and REEl) made from CAD files
* blender_script added to export .fbx cad files to daes
* Scripts and services added to allow the importing of worlds from the gazebosim_world_builder and to generate worlds from a folder of dae files i.e. for use with the blender script.
* Services added to allow for automated world degrading and random selection of models to be degraded. 

-------

## Important note!!!

This repository borrows a script called custom sensor preloader from: https://github.com/peci1/gazebo_custom_sensor_preloader.

I should just make this depend on that package but I get compile issues when I try to do this.(Most likely user error).   

**I am not trying to take credit for peci1's coding of the preloader!**

-------

## Install Requirements

Please make sure you are installing the correct branch. The master is Kinetic/Gazebo7 with a branch for Melodic/Gazebo9 (which also seems to work with Noetic)


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

#### Plugins

##### CustomSensorPreloader.cpp/h 
for loading custom sensors into gazebo 
##### RadiationSensor.cpp/h
creates a gazebo radiation sensor 
##### RadiationSource.cpp/h
creates a gazebo radiation source 
##### RadiationSensorPlugin.cpp/h
creates a plugin interface between ros and gazebo for the sensors
##### RadiationSourcePlugin.cpp/h 
creates a plugin interface between ros and gazebo for the sources

#### Scripts

##### gazebo.in
runs gazebo with the customsensorpreloader script

When using the custom sensors with gazebo you need to use rosrun gazebo_radiation_plugin gazebo(/gzserver)
##### gzserver.in
runs gzserver with the customsensor script
##### generate_radiation_sources.py
creates example radiation sources from rosparameters (requires the sources.yaml file to be loaded into the rosparam server)
##### load_radiation_sources.py
Adds each of the example sources into gazebo from the folder specified sources_folder.

##### mass_yaml_loader.py
Used to load all yaml files in a folder onto the rosparam server. 
This is used to load all of the yaml files used by the model_evolver service onto the server.

#### Non-ROS

##### generate_model_list.py

Python script used ot generate the model lists when a world has been made using a CAD file rather than the world builder. 
As the CAD files are never passed through the convert_world_builder_model service these files are not generated.

This shoudl be combined into the generate_world_from_models services in the future. 

##### blender_script/export_to_dae.py

Shold be run through blender.

It allows for CAD models to be exported to a folder where each element is its own .dae file


#### Services

**Note for many of these services to work, the template folder structure found in the custom_models folder should be adhered to!**

Below are all of the services, their required input arguments and what they do.

##### convert_world_builder_model.py

Imports a model made using the gazebosim_world_builder (or any other gazebo .world file) into a folder specified. This will create duplicate models for each item in the model which share a common model to allow for deformation. 
If a noise file is specied some initial deformation is applied to each model as to make them all unique. 

See media/world_builder_to_model_evolver for reference.

* world_builder_file: The location of the world to be imported
* gazebo_model_path: Where to create the new model. This should be in one of the gazebo_model_paths, which can be specified in the package.xml export tags.
* noise_file: the location of the noise file. This allows random deformations to be applied to each of the models to make them unique. An example file is found in custom_models/template/yamls/models_with_noise_example.yaml. This causes all of the items which use the model called barrel.dae to be deformed slightly physically and in colour.
  
This service also produced the all/bendable/rustable_models.txt files which is placed into the models folder. These files contain lists of all of the which have been found to be useable for:

adding radiation: all_models.txt
be bendable: bendable_models.txt
be rustable: rustable_models.txt

##### generate_random_environment_effects.py

This service will randomly select models to add radiation sources to based on the all/bendable/rustable_models.txt files in the models folder. If models exist in these files you do not want considering e.g. walls. Just delete them from the files

Once called this service will produce a file called subset.yaml in the yaml folder of the world. which contains the details of which models have been selected for radiation/bending/rusting and how much each will be changed at each evolution. These can be manually altered if required.

* folder: The folder of the custom world  e.g. path/to/custom_models/exampe_world/
* number_of_radiation_sources: Desired number of radiation sources to be added
* number_of_rusting_models: Desired number of rusting models to be altered
* number_of_bending_models: Desire number of expanding models to be altered
* max_bend_factor: The maximum amount of bend allowed at each evolution as a decimal. i.e. 1% would be 0.01
* max_rust_factor: The maximum amount of rust to be added at each evolution as a decimal.

##### generate_world_from_models.py

If an enviroment is created using the blender script. The template folder in the custom_models folder should be copied and renamed. Then the produced .dae files shoudl be put in the models folder within the new folder. i.e. custom_models/*NEW_MODEL_NAME*/models/ . 

Once copied this service can be invoked to produce a world file based on all of the models within the folder.  

* folder: The folder to be used for the new custom world e.g. path/to/custom_models/example_world/
* model_filename The name of the output world. e.g. new_model.world

##### generate_yamls_for_world.py

This service will generate all of the yaml files used by the model_evolver service based on the subset.yaml file.

Once called this service will produce a number of yaml files placed into the yaml folder of the environment. These will need to be loaded onto the rosparam server for use with the model_evolver service. This can be done manually or by using the mass_yaml_loader script.

* folder: the location of the custom_world folder. e.g. path/to/custom_models/example_world/
* world_filename: the filename of the world file no path required. 
* model_subset_filename: the location of the subset file: e.g. yamls/subset.yaml
* default_bend: the default value used if one is not specified int he subset.yaml file. 
* default_rust: the default value used if one is not specified int he subset.yaml file.
* default_radiation: the default value used if one is not specified int he subset.yaml file.

##### model_evolver.py

Used to evolve the environment based on the yaml files loaded on the rosparam server.

##### model_mover.py
For testing. This service simply rotates a model called sensor in a circle. See media/gazebo_radiation_sensor_and_sources for details 

### Additional Settings

The radiation sensor has some additional settings which can be set on the param server these are type,range,collimated,mu,sigma and attenuation_factors.

For each sensor type and range should be loaded using /SENSOR_NAME/type and /SENSOR_NAME/range where SENSOR_NAME is the sensor name e.g. sensor_0. These parameters allow you to set maximum sensing ranges and radiation types the sensor is detecting . e.g. Alpha, Beta, Gamma; if the sensor is collimated and if it is collimated define the mu and sigma of the (gaussian)sensitivity function. If these are set the sensor will disregard anything too far away or of the wrong radiation type and reduce the measured radiation in accordance with the angle of incidency using the sensitivity function. 

Attenuation_factors is set using a yaml file and is explained in the running the example section.

### Running the example

The easiest way to test if this reposity is working for you is to simply source the workspace and roslaunch the radiation_demonstrator.launch. From here you should then be able to echo out the values coming from the radiatin sensor. 

Alternatively if you have the radiation layer installed the values can be painted into an occupancy grid by:   
1. Install the radiation layer in your catkin workspace 
2. In the radiation_demonstrator launch file you will find a launch file commented out called radmap.launch. This needs uncommenting.
3. Replace the radmap_params.yaml file in launch/params/ with radiation layer yaml file.
4. Add the radiation occupancy grid to rviz for viewing.

A simple version of the radiation layer can be found here: https://github.com/EEEManchester/simple_radiation_layer


Alternatively if you wish to run an example step by step you can follow the below steps to build a simple environment from scratch.

The first thing which needs to be done is load some parameters into the rosparam server which relate to the radiation sources being used. Basic sources can be described in the format shown in configs/sources.yaml requiring x,y,z,type and value,noise and units. Use:
 ```
roscore
 ```
Then load the sources yaml file located in the configs folder by cd'ing into the folder and running
 ```
rosparam load sources.yaml
rosparam load sensors.yaml 

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
rosrun gazebo_ros spawn_model -file src/gazebo_radiation_plugin/sdf/radiation_sensor.sdf  -sdf -x 0 -y 0 -z 0.1 -model sensor_0
```
(for real use this sensor would be attached to a robot).

a script to move the example sensor is included. Or alternatively you can simply drag it around inside gazebo.
```
rosrun gazebo_radiation_plugins model_mover.py
```

Now you should be able to see the rostopic showing up for the sensor and how the value changes as you move it around. NB: if collomation is being use then the green square represents the front of the sensor and thus the sensor will need rotating around to face the sources. 


# The Physics

Sources of radiation are modelled as point sources, with constant activity (decays per second).  The intensity of radiation as a function of position in a simulated environment is based on two phenonema, the fraction of radiation impinging on a psuedo-detector and shielding due to objects in the environment.

## Modeling a Radiation Sensor

At a distance, ```D```, from a point radioactive source the number of radiation interactions, ```I_D```,  which occur for a circular sensor, when facing normal to the source is described by:

```I_D = 1/2 * I_0 * ( 1- ( D / sqrt(D^2 + R^2) ) )```

where ```I_0``` is the activity of the source, ```D``` is the distance between the sensor and the source, and ```R``` is the radius of the circular sensor face.  As ```D >> R```, the relationship reduces to the commonly known inverse distance squared law (```I``` proportional to ```1/D^2```).  

When the sensor is in very close proximity to a source (```D -> 0```), the total interactions is half the total.  This is more physically accurate that other implementations which only use an inverse distance squared approach (which tends to infinity).

## Attenuation

Between a source and the detector, materials act to absorb, scatter and generally attenuate the expected intensity.   For a given thickness of material ```Z```, and linear attenuation coefficient ```a```, the ration of measured intensity, ```I``` compared to the expected intensity ```I_D``` is described by the Beer-Lambert Law:

```I / I_D = exp(-aZ)```

Given an imaginary ray between a source and the detector, all objects (including air) and the distance travelled through that medium needs to be recorded.  The total attenuation through ```j``` objects is the product of all the individual attenuation contributions described by the Beer-Lambert Law.

This yields the measured intensity at distance ```I_D```  to be:

```I_D = [ 1/2 * I_0 * ( 1- ( D / sqrt(D^2 + R^2) ) ) ] * PRODUCT_j(exp(-a_j * Z_j))```

## Sensor Sensitivity

A real-world detector is often more sensitive to radiation in particular directions, either due to the geometry of the detecting volume, or due to other materials attenuation radiation (such as the detector casing or deliberate collimation with lead/tungsten).

This sensitivity can either be explicitly defined, or approximated ysing a convenient function such as a Gaussian or Uniform distribution.  Given the angle between the detector face and the source, ```angle```, this yields a value between 0-1 for the sensitivity based on function ```f(angle)```.  Therefore, the intensity measured by a detector is further corrected to give:

```I_D = f(angle) * [ 1/2 * I_0 * ( 1- ( D / sqrt(D^2 + R^2) ) ) ] * PRODUCT_j(exp(-a_j * Z_j))```


## Multiple Radiation Sources

An instrument of uniform sensitivity will detect photons or heavy particles from radioactive decay regardless of source, therefore multiple sources will provide a contribution to the total radiation flux impinging on the pseudo-sensor.

However, the flux from each source is modified based on attenuation by objects and the sensitivity with respect to the sensor orientation.  The total intensity then is the summation of all contributions from different sources given their distance, attenuation, and sensitivity.

```I_D_Total = SUM [ f(angle) * [ 1/2 * I_0 * ( 1- ( D / sqrt(D^2 + R^2) ) ) ] * PRODUCT_j(exp(-a_j * Z_j))]```

## Variance of Radioactive Decay

Radioactive decay is a naturally random process.  Though the average activity of a source can be estimated, for a given period of time the number of decay events will fluctuate, and this fluctuation can be described by a Poisson Distribution.

The radiation intensity impinging on pseudo-detector (by multiple sources), can be augmented to include this fluctuation expected from real-world radioactive materials.  The total radiation intensity is used as the mean of a Poisson distribution, and a random integer number of events are drawn from this distribution.

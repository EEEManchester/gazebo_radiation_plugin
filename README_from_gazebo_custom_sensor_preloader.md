# Implement custom Gazebo sensors at ease

The current structure of the Gazebo simulator doesn't allow 
implementation of custom `<sensor>`s in the form of externally loaded
plugins. To add a new sensor implementation, you officially need
to fork Gazebo and add the sensor to its source code.

This doesn't sound really great, does it?

This Gazebo system plugin allows you to write custom Gazebo sensors
as ROS packages (so it depends on 
[gazebo_ros](//github.com/ros-simulation/gazebo_ros_pkgs), and adding them
to Gazebo is then a matter of a few configuration lines in your sensor
code. Theoretically, the mechanism this plugin uses could work
completely without ROS, but hey, who uses Gazebo without ROS? :)

This plugin is only tested to work with Gazebo 9. If you successfully use it
with a different version, please let me know in the issues.

## Known custom sensors

Here's a (noncomprehensive) list of known custom sensor implementations that
work with this plugin. Feel free to open a pull request to add your own implementation
here.

* [rotating_lidar_sensor](//github.com/peci1/rotating_lidar_sensor): A sensor for
more realistic simulation of lidars based on a rotating mirror where each laser
beam has a different timestamp.

## How to do it

There is an example custom sensor in this package: 
[ExampleCustomSensor.cpp](src/ExampleCustomSensor.cpp),
[ExampleCustomSensor.h](include/gazebo_radiation_plugins/ExampleCustomSensor.h)
and [example_custom_sensor.xml](example_custom_sensor.xml).
The most important things will be described further in this document.

> ⚠ In this guide, we use the names `ExampleCustomSensor` and
>`example_custom_sensor`, which you have to change, because a custom sensor 
>with this class/name is already built in this package.

### Create a ROS/catkin package for your sensor

E.g. by calling

    catkin_create_pkg ... example_custom_sensor ...

### A pretty normal `Sensor` implementation ...

#### ExampleCustomSensor.h

Here, it is important to note that your custom sensor *has to* reside inside the
`gazebo::sensors` namespace.

```c++
#include <gazebo/sensors/Sensor.hh>

namespace gazebo
{
namespace sensors
{

class ExampleCustomSensor : public Sensor
{

// your code

}
}
}
```

#### ExampleCustomSensor.cpp

In the implementation file, you have to register your sensor via the following
block of code. The first argument is the Gazebo sensor type, which is how you
reference the custom sensor in SDF. It should also match the `name` attribute 
in [XML plugin definition](example_custom_sensor.xml) (prefixed with `sensors/`).

```c++
#include <gazebo/sensors/SensorFactory.hh>

using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;
extern "C"
{
GZ_REGISTER_STATIC_SENSOR("example_custom_sensor", ExampleCustomSensor)
}

// your sensor implementation
```

### ... a few configuration lines ...

#### example_custom_sensor.xml

This is a configuration file you may know if you've ever used ROS 
[pluginlib](//github.com/ros/pluginlib), e.g. when implementing a nodelet.
The library path is relative to the devel space of your sensor's workspace,
and contains the name of the shared object containing the sensor, excluding the
`.so` extension. Class name is the same you used as the first argument to
`GZ_REGISTER_STATIC_SENSOR`, prefixed with `sensors/` to avoid name collisions.
Class type is the fully qualified C++ name of your sensor's class, and base
class type should always be `gazebo::sensors::Sensor`.

```xml
<library path="lib/libexample_custom_sensor">
  <class name="sensors/example_custom_sensor"
         type="gazebo::sensors::ExampleCustomSensor"
         base_class_type="gazebo::sensors::Sensor">
    <description> 
      An example Gazebo custom sensor skeleton.
    </description>
  </class>
</library>
```

#### package.xml

In [package.xml](package.xml), you just have to `<exec_depend>` on this package
and add an `<export>` tag that specifies the path to the above-created XML file.
`${prefix}` will get expanded to your package's source directory root.

```xml
<package>
  ...
  <exec_depend>gazebo_radiation_plugins</exec_depend>
  <export>
    <gazebo_radiation_plugins plugin="${prefix}/example_custom_sensor.xml" />
  </export>
</package>
```

#### CMakeLists.txt

```cmake
...
catkin_package(
  ...
  CATKIN_DEPENDS ... gazebo_radiation_plugins
  ...
)
...
install(FILES example_custom_sensor.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
```

### ... and don't forget to load this system plugin

This is the greatest pain in the ***, but it has to be done. Whenever you launch
Gazebo and want to use a custom sensor, you need to start it with the server 
plugin path like this:

```shell script
gzserver -s /absolute/path/to/libgazebo_radiation_plugins.so "other" "args"
```

This package provides a convenience script which combines this system plugin with
`gazebo_ros` system plugins into a single commandline, so that you can run gazebo
with

```shell script
rosrun gazebo_radiation_plugins gzserver "other" "args"
```

But this is just a convenience script, you can as well implement your own custom
Gazebo launcher script.

### Use it!

#### my_great_robot.sdf

The sensor `type` attribute contains the same value you used as the first argument
to `GZ_REGISTER_STATIC_SENSOR`.

```xml
<sdf ...>
  <model ...>
    <link ...>
      <sensor name="great_sensor" type="example_custom_sensor">
      </sensor>
    </link>
  </model>
</sdf>
```

## Debugging

If everything is set up correctly, you should see at least the following lines in
the console if your launch gzserver with the `--verbose` flag:

    [Msg] CustomSensorPreloader: Preloaded custom sensor ExampleCustomSensor from library /some/path/lib/libexample_custom_sensor.so
    [Msg] CustomSensorPreloader: Adding GAZEBO_PLUGIN_PATH /some/path/lib
    
If the custom plugin preloader is missing or wrongly configured, you'll see instead

    [Err] [SensorManager.cc:295] Unable to create sensor of type[your_custom_sensor]

## How it works

This plugin makes use of the fact that function `GZ_REGISTER_STATIC_SENSOR` creates
a global function `RegisterCustomPlugin()` in the sensor's shared library, which
basically calls `SensorFactory::RegisterSensor(name, NewCustomPlugin)`. So this
plugin just finds the right shared libraries and calls these registration functions
in each of them.

Unfortunately, of all the types of Gazebo plugins (Sensor, Model, World and System),
only the System plugins can run the registration code early enough so that Gazebo
finds the custom sensor implementation.
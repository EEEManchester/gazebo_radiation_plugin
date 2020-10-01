
#include <gazebo_radiation_plugins/RadiationSourcePlugin.h>

namespace gazebo
{


  RadiationSourcePlugin::RadiationSourcePlugin():
  _nh("radiation_source_plugin")
  {

  }


  RadiationSourcePlugin::~RadiationSourcePlugin()
  {
    ROS_DEBUG_STREAM_NAMED("Radiation","stopped");
  }

  void RadiationSourcePlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
  {
      
      
      // Make sure the ROS node for Gazebo has already been initialized                                                                                    
      if (!ros::isInitialized())
      {
        ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
          << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
      }

    this->parentSensor =std::dynamic_pointer_cast<sensors::RadiationSource>(_sensor);

    // Make sure the parent sensor is valid.
    if (!this->parentSensor)
    {
      gzerr << "No radiation source.\n";
      return;
    }

    // Connect to the sensor update event.
    this->updateConnection = this->parentSensor->ConnectUpdated(
        std::bind(&RadiationSourcePlugin::OnUpdate, this));
  
    _publisher = _nh.advertise<gazebo_radiation_plugins::Simulated_Radiation_Msg>(this->parentSensor->topic, 10);

  
  }

  void RadiationSourcePlugin::OnUpdate()
  {


    gazebo_radiation_plugins::Simulated_Radiation_Msg msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "sim_world";
    msg.value = this->parentSensor->radiation;
    msg.pose.position.x = this->parentSensor->pose.Pos()[0];
    msg.pose.position.y = this->parentSensor->pose.Pos()[1];
    msg.pose.position.z = this->parentSensor->pose.Pos()[2]; 
    msg.pose.orientation.x = this->parentSensor->pose.Rot().X();
    msg.pose.orientation.y = this->parentSensor->pose.Rot().Y();
    msg.pose.orientation.z = this->parentSensor->pose.Rot().Z();
    msg.pose.orientation.w = this->parentSensor->pose.Rot().W();
    
    msg.type = this->parentSensor->radiation_type;

    this->_publisher.publish(msg);


  }

}

GZ_REGISTER_SENSOR_PLUGIN(gazebo::RadiationSourcePlugin);
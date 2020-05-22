#ifndef gazebo_radiation_plugins_RadiationSourcePLUGIN_H
#define gazebo_radiation_plugins_RadiationSourcePLUGIN_H

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/advertise_options.h>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Point.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <gazebo_radiation_plugins/RadiationSource.h>
#include <gazebo_radiation_plugins/Simulated_Radiation_Msg.h>


namespace gazebo
{

class RadiationSourcePlugin : public SensorPlugin
{
  /// brief Constructor
  /// param parent The parent entity, must be a Model or a Sensor
  public: RadiationSourcePlugin();

  /// brief Destructor
  public: ~RadiationSourcePlugin();

  public: void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

  /// brief Update the controller
  protected: void OnUpdate();

  protected: ros::NodeHandle _nh;

  protected: ros::Publisher _publisher;

  /*
  protected: ignition::math::Pose3d _pose;
  protected: double _value;
  */

  private: std::shared_ptr<sensors::RadiationSource> parentSensor;

  /// \brief Connection that maintains a link between the contact sensor's
  /// updated signal and the OnUpdate callback.
  private: event::ConnectionPtr updateConnection;


};

}

#endif //gazebo_radiation_plugins_RadiationSourcePLUGIN_H

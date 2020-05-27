#ifndef gazebo_radiation_plugins_RadiationSensor_H
#define gazebo_radiation_plugins_RadiationSensor_H


#include "gazebo/util/system.hh"
#include <gazebo/common/Plugin.hh>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/PhysicsTypes.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include <gazebo/transport/transport.hh>
#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/msgs/msgs.hh"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include "gazebo/sensors/SensorManager.hh"


#include <gazebo/common/Events.hh>

#include <sdf/sdf.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"

#include <gazebo_radiation_plugins/RadiationSource.h>

#include <ignition/math/Pose3.hh>

#include <memory>
#include <math.h>


// It is very important that your custom sensor class is defined inside the
// gazebo::sensors namespace.

namespace gazebo
{
namespace sensors
{
  

class GAZEBO_VISIBLE RadiationSensor : public Sensor
{
  public: RadiationSensor();
  public: ~RadiationSensor() override;
  public: virtual void Load(const std::string &_worldName, sdf::ElementPtr _sdf) override;
  public: virtual void Load(const std::string & _worldName);
  public: void Init() override;
  protected: bool UpdateImpl(bool force) override;
  protected: void Fini() override;


  public: sdf::ElementPtr GetSDF();
  public: void AddSource(RadiationSource *_rs);
  public: void RemoveSource(std::string);
  private: void EvaluateSources();

  private: double CheckSourceRange(const ignition::math::Pose3d &_pose);
  private: double CheckSourceAngle(const ignition::math::Pose3d &_pose);


  public: ignition::math::Pose3d GetPose() const;
  public: ignition::math::Pose3d pose;

  public: physics::EntityPtr entity;

  private: event::ConnectionPtr updateConnection;
    
  public: transport::PublisherPtr scanPub_pose;
  public: transport::PublisherPtr scanPub_value;

  public: double radiation;
  public: std::string topic;
  public: std::string sensor_type; 
  

  public: std::vector<RadiationSource*> sources;

  public: ros::NodeHandle n;
  private: XmlRpc::XmlRpcValue params;

  private: double sensitivity_function(double);

  private: double mu = 0.0;
  private: double sig = 0.2;


};

}
}

#endif //gazebo_radiation_plugins_RadiationSensor_H

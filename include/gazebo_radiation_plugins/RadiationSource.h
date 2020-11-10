#ifndef gazebo_radiation_plugins_RadiationSource_H
#define gazebo_radiation_plugins_RadiationSource_H


#include "gazebo/util/system.hh"
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/PhysicsTypes.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include <gazebo/transport/transport.hh>
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/physics/WorldState.hh"

#include "gazebo/msgs/msgs.hh"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorFactory.hh>
#include "gazebo/sensors/SensorManager.hh"



#include <sdf/sdf.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"



#include <ignition/math/Pose3.hh>

#include <memory>

// It is very important that your custom sensor class is defined inside the
// gazebo::sensors namespace.

namespace gazebo
{
namespace sensors
{
  

class GAZEBO_VISIBLE RadiationSource : public Sensor
{
  public: RadiationSource();
  public: ~RadiationSource() override;
  public: virtual void Load(const std::string &_worldName, sdf::ElementPtr _sdf) override;
  public: virtual void Load(const std::string & _worldName);
  public: void Init() override;
  protected: bool UpdateImpl(bool force) override;
  protected: void Fini() override;

  public: ignition::math::Pose3d GetPose() const;
  public: sdf::ElementPtr GetSDF();
  public: void OnUpdate();

  public: ignition::math::Pose3d pose;
  public: double radiation;
  public: std::string radiation_type;
  public: std::string name;
  public: std::string topic;
  public: std::string units;
  public: double noise;

  public: physics::EntityPtr entity;


  public: transport::PublisherPtr radPub_pose;
  public: transport::PublisherPtr radPub_value;
  public: transport::PublisherPtr radPub_type;

  public: SensorPtr sensor;
  public: bool gotSensor = false;

  public: ros::NodeHandle n;
  private: XmlRpc::XmlRpcValue params;
  //private: std::unique_ptr<ros::NodeHandle> rosNode;
 private: event::ConnectionPtr updateConnection;
    

};

}
}

#endif //gazebo_radiation_plugins_RadiationSource_H

#ifndef gazebo_radiation_plugins_RadiationSensor_H
#define gazebo_radiation_plugins_RadiationSensor_H


#include "boost/pointer_cast.hpp"

#include "gazebo/util/system.hh"
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/RayShape.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/WorldState.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"
#include <gazebo/transport/transport.hh>
#include "gazebo/transport/TransportTypes.hh"

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

#include <gazebo_radiation_plugins/RadiationSource.h>

#include <ignition/math/Pose3.hh>

#include <memory>
#include <math.h>
#include <random>  // Used for Poisson statistics


// It is very important that your custom sensor class is defined inside the
// gazebo::sensors namespace.

struct raySegment {
  double length;
  std::string from;
  std::string to;
  raySegment(double d,  std::string s0, std::string s1) : length(d),from(s0),to(s1) { }
} ;

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
  private: double CheckCollimation(double,const ignition::math::Pose3d &_pose);
  private: std::vector<raySegment> CheckSourceViewable(ignition::math::Vector3d,ignition::math::Vector3d, std::string);
  private: double AttenuationFactor(std::vector<raySegment>);

  public: ignition::math::Pose3d GetPose() const;
  public: ignition::math::Pose3d pose;

  public: physics::EntityPtr entity;

  private: event::ConnectionPtr updateConnection;
    
  public: transport::PublisherPtr radPub_pose;
  public: transport::PublisherPtr radPub_value;

  public: std::vector<RadiationSource*> sources;
  public: SensorPtr sensor;
  public: bool gotSensor = false;

  public: ros::NodeHandle n;
  private: XmlRpc::XmlRpcValue params;

  private: double sensitivity_function(double);
  private: double SolidAngle(double);

  // User inputs and defaults
  public: double radiation;
  public: std::string topic;
  public: std::string sensor_type; 
  public: double sensor_range = 1000000000.0;
  private: double mu = 0.0;
  private: double sig = 1.0;
  private: double collimation_attenuation = 0.0;
  private: std::string collimated = "";
  private: double angle_limit = 0.0;
  private: double az_limit = 0.0;
  private: double el_limit = 0.0;
  private: std::string sensitivity_func = "";
  private: bool poisson = false;
  private: std::vector<std::pair<float,float> > frustum_points;
  public: physics::RayShapePtr blockingRay;
  private: std::pair<float,float>  calc_equations(std::pair<float,float>,std::pair<float,float>);

  private: XmlRpc::XmlRpcValue attenuation_factors;

  private: std::default_random_engine generator;  // Random number generator for Poisson distribution

  private: int radCount = 0;

};

}
}

#endif //gazebo_radiation_plugins_RadiationSensor_H

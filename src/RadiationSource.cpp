#include <gazebo_radiation_plugins/RadiationSource.h>
#include <gazebo_radiation_plugins/RadiationSensor.h>

//DESTRUCTOR/CONSTRUCTOR  NEEDS TO SEARCH FOR SENSORS AND ADD/REMOVE SOURCES!!
//ADD NOISE TO SOURCE

// Do not forget to register your sensor via this block of code.
// The first argument is the Gazebo sensor type, which is how you reference the
// custom sensor in SDF. It should also match the 'name' attribute in XML plugin
// definition (together with the 'sensors/' prefix).
using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;

extern "C"
{
  GZ_REGISTER_STATIC_SENSOR("radiation_source", RadiationSource)
}

// you can also use other sensor categories
gazebo::sensors::RadiationSource::RadiationSource()
    : Sensor(gazebo::sensors::SensorCategory::OTHER)
{
  this->active = true;
}

gazebo::sensors::RadiationSource::~RadiationSource()
{
}

/////////////////////////////////////////////////
void gazebo::sensors::RadiationSource::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}

void gazebo::sensors::RadiationSource::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&gazebo::sensors::RadiationSource::UpdateImpl, this, true));

  if (this->sdf->GetElement("topic"))
  {
    this->topic = this->sdf->GetElement("topic")->Get<std::string>();
    this->name = this->topic.substr(0, this->topic.find("/"));
    this->radiation_type = this->topic.substr(this->name.size() + 1, this->topic.find("/"));

    this->radPub_pose = this->node->Advertise<msgs::Pose>(this->name + "/pose");
    this->radPub_value = this->node->Advertise<msgs::Any>(this->name + "/value");
    this->radPub_type = this->node->Advertise<msgs::Any>(this->name + "/type");

    this->entity = this->world->EntityByName(this->ParentName());

    n.getParam("/sources/" + this->name, params);

    this->radiation = static_cast<double>(params["value"]);
    this->units = static_cast<std::string>(params["units"]);
    this->noise = static_cast<double>(params["noise"]);

    gzwarn << "PARAMS LOADED" << std::endl;

    Sensor_V sensors = SensorManager::Instance()->GetSensors();
    for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter)
    {

      //gzmsg << (*iter)->Type() << std::endl;
      if ((*iter)->Type() == "radiation_sensor")
      {
        std::dynamic_pointer_cast<RadiationSensor>(*iter)->AddSource(this);
      }
    }

    gzmsg << "spawning source " << this->name << " of type " << this->radiation_type << " with value " << this->radiation << std::endl;
  }
}

void gazebo::sensors::RadiationSource::Fini()
{

  {
    boost::recursive_mutex::scoped_lock lock(*(
        this->world->Physics()->GetPhysicsUpdateMutex()));

    Sensor_V sensors = SensorManager::Instance()->GetSensors();
    for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter)
    {

      //gzmsg << (*iter)->Type() << std::endl;
      if ((*iter)->Type() == "radiation_sensor")
      {
        std::dynamic_pointer_cast<RadiationSensor>(*iter)->RemoveSource(this->name);
      }
    }

    //this->entity.reset();
    Sensor::Fini();
  }
}

void gazebo::sensors::RadiationSource::Init()
{
  Sensor::Init();
}

bool gazebo::sensors::RadiationSource::UpdateImpl(const bool force)
{

  if (this->gotSensor == false)
  {
    Sensor_V sensors = SensorManager::Instance()->GetSensors();
    for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter)
    {
      if ((*iter)->Type() == this->Name())
      {

        this->sensor = (*iter);
        this->gotSensor = true;
      }
    }
  }

  msgs::Pose msg_pose;
  this->pose = this->GetPose();
  msgs::Set(&msg_pose, pose);

  msgs::Any msg_value;

  if (this->noise != 0.0)
  {
    msg_value = msgs::ConvertAny(this->radiation + rand() / (RAND_MAX / this->noise));
  }
  else
  {
    msg_value = msgs::ConvertAny(this->radiation);
  }
  msgs::Any msg_type;
  msg_type = msgs::ConvertAny(this->radiation_type);

  this->radPub_pose->Publish(msg_pose);
  this->radPub_value->Publish(msg_value);
  this->radPub_type->Publish(msg_type);

  return true;
}

ignition::math::Pose3d gazebo::sensors::RadiationSource::GetPose() const
{
  ignition::math::Pose3d p;

  if (this->gotSensor)
  {
    p = this->sensor->Pose() + this->entity->WorldPose();
  }
  else
  {
    p = this->entity->WorldPose();
  }
  return p;
}

// void gazebo::sensors::RadiationSource::SetPose() const
// {
//   //from rosparam server!!!
//   return this->entity->SetWorldPose();s
// }

sdf::ElementPtr gazebo::sensors::RadiationSource::GetSDF()
{
  return this->sdf;
}

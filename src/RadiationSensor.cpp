#include <gazebo_radiation_plugins/RadiationSensor.h>
#include <gazebo_radiation_plugins/RadiationSource.h>

// ADD A WAY OFF DECIDING OF ITS COLLIMATED OR NOT/Type of sensor 

//maybe include ros and do rosparam

// Do not forget to register your sensor via this block of code.
// The first argument is the Gazebo sensor type, which is how you reference the
// custom sensor in SDF. It should also match the 'name' attribute in XML plugin
// definition (together with the 'sensors/' prefix).
using gazebo::sensors::Sensor;
using gazebo::sensors::SensorFactory;


extern "C"
{
GZ_REGISTER_STATIC_SENSOR("radiation_sensor", RadiationSensor)
}

// you can also use other sensor categories
gazebo::sensors::RadiationSensor::RadiationSensor()
    : Sensor(gazebo::sensors::SensorCategory::OTHER)
{
  this->active = true;
}

gazebo::sensors::RadiationSensor::~RadiationSensor() 
{
}


/////////////////////////////////////////////////
void gazebo::sensors::RadiationSensor::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);
}


void gazebo::sensors::RadiationSensor::Load(const std::string &_worldName)
{
  Sensor::Load(_worldName);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&gazebo::sensors::RadiationSensor::UpdateImpl, this,true));


  if (this->sdf->GetElement("topic"))
  {
    this->scanPub_pose = this->node->Advertise<msgs::Pose>(this->sdf->GetElement("topic")->Get<std::string>()+"/pose");
    this->scanPub_value = this->node->Advertise<msgs::Any>(this->sdf->GetElement("topic")->Get<std::string>()+"/value");
  }
    
  this->topic = this->sdf->GetElement("topic")->Get<std::string>();

  //this->sensor_type = this->sdf->GetElement("sensor_type")->Get<std::string>();
  this->sensor_type = "gamma";

  this->entity = this->world->GetEntity(this->ParentName());

  
  // Add the tag to all the RFID sensors.
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter)
  {

    //gzmsg << (*iter)->Type() << std::endl;
    if ((*iter)->Type() == "radiation_source")
    {
      
      auto sensorPtr = std::static_pointer_cast<RadiationSource>(*iter).get();
      this->AddSource(sensorPtr);
    }
  }
  
}

void gazebo::sensors::RadiationSensor::Fini()
{
  Sensor::Fini();
  this->entity.reset();
}

void gazebo::sensors::RadiationSensor::Init()
{
  Sensor::Init();
}

bool gazebo::sensors::RadiationSensor::UpdateImpl(const bool force)
{

    msgs::Pose msg_pose;
    this->pose = this->GetPose();
    msgs::Set(&msg_pose, pose);

    this->EvaluateSources();
    msgs::Any msg_value;
    msg_value = msgs::ConvertAny(this->radiation);

    this->scanPub_pose->Publish(msg_pose);
    this->scanPub_value->Publish(msg_value);   


    return true;
}

//////////////////////////////////////////////////
void gazebo::sensors::RadiationSensor::EvaluateSources()
{
  std::vector<RadiationSource*>::const_iterator ci;

  // iterate through the tags contained given rfid tag manager
  this->radiation = 0.0;

  for (ci = this->sources.begin(); ci != this->sources.end(); ci++)
  {
    ignition::math::Pose3d pos = (*ci)->GetPose();
    std::string type = (*ci)->radiation_type;
    double value = (*ci)->radiation;
    double dist = this->CheckSourceRange(pos);
    this->radiation += value/((dist*dist)+(+3.3E-5*3.3E-5));
    //gzmsg << this->radiation << "," << value << "," << dist << std::endl;

  }


}

//////////////////////////////////////////////////
double  gazebo::sensors::RadiationSensor::CheckSourceRange(const ignition::math::Pose3d &_pose)
{
  // copy sensor vector pos into a temp var
  ignition::math::Vector3d v;
  v = _pose.Pos() - this->entity->GetWorldPose().Ign().Pos();

  // std::cout << v.GetLength() << std::endl;

  return v.Length();
}

//////////////////////////////////////////////////
ignition::math::Pose3d gazebo::sensors::RadiationSensor::GetPose() const
{
  return this->entity->GetWorldPose().Ign();
}

void gazebo::sensors::RadiationSensor::AddSource(RadiationSource *_rs)
{
  if ( _rs->radiation_type == this->sensor_type){
    gzmsg << "adding source" << std::endl;
    this->sources.push_back(_rs);
  }
}

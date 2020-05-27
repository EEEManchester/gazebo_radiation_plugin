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

/*
The only way to add custom elements to the sdf is to dig into the the sdf definintions
Pull stuff from ros param server !!! like radiation
Write a script to get the radiation source file from the output yaml!!!
*/

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
  this->sensor_type = "";

  this->entity = this->world->GetEntity(this->ParentName());

  if (n.hasParam(this->topic+"/type")){
      n.getParam(this->topic+"/type",this->sensor_type);
    } 

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

  double rad = 0.0;
  for (ci = this->sources.begin(); ci != this->sources.end(); ci++)
  {
    ignition::math::Pose3d pos = (*ci)->GetPose();
    std::string type = (*ci)->radiation_type;
    double value = (*ci)->radiation;
    double dist = this->CheckSourceRange(pos);
    double sensitivity = sensitivity_function(this->CheckSourceAngle(pos)/2.0);
    //gzmsg << (*ci)->name << " " << sensitivity << std::endl;
    rad += sensitivity*value/((dist*dist)+(+3.3E-5*3.3E-5));
    float r = rand()/RAND_MAX;

    if (r < (rad - floor(rad))){
      this->radiation = floor(rad);
    } else {
      this->radiation = ceil(rad);
    }
    //gzmsg << this->radiation << "," << value << "," << dist << std::endl;

  }


}

double gazebo::sensors::RadiationSensor::sensitivity_function(double x){
    return exp(-pow(x - this->mu, 2.0) / (2.0* pow(this->sig, 2.0)));
}

//////////////////////////////////////////////////
double  gazebo::sensors::RadiationSensor::CheckSourceRange(const ignition::math::Pose3d &_pose)
{
  // copy sensor vector pos into a temp var
  ignition::math::Vector3d v;
  v = _pose.Pos() - this->entity->GetWorldPose().Ign().Pos();

  //gzmsg << this->entity->GetWorldPose().Ign().Pos()[0] << " " << this->entity->GetWorldPose().Ign().Pos()[1] << " " << this->entity->GetWorldPose().Ign().Pos()[2] << std::endl;  
  //gzmsg << _pose.Pos()[0] << " " << _pose.Pos()[1] << " " << _pose.Pos()[2] << std::endl;
  //gzmsg << v[0] << " " << v[1] << " " << v[2] << std::endl;
  //gzmsg << v.Length() << std::endl;
  return v.Length();
}
//////////////////////////////////////////////////
float dot(ignition::math::Vector3d a, ignition::math::Vector3d b)  //calculates dot product of a and b
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float mag(ignition::math::Vector3d a)  //calculates magnitude of a
{
    return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

double gazebo::sensors::RadiationSensor::CheckSourceAngle(const ignition::math::Pose3d &_pose)
{
  // copy sensor vector pos into a temp var
  ignition::math::Vector3d v0;
  ignition::math::Vector3d v1;
  ignition::math::Vector3d v(1,0,0);
  
  v0 = _pose.Pos()- entity->GetWorldPose().Ign().Pos();
  v1 = entity->GetWorldPose().Ign().Rot().RotateVector(v);
  
  double angle = std::acos(dot(v0,v1)/(mag(v0)*mag(v1)));
  //gzmsg << this->entity->GetWorldPose().Ign().Pos()[0] << " " << this->entity->GetWorldPose().Ign().Pos()[1] << " " << this->entity->GetWorldPose().Ign().Pos()[2] << std::endl;  
  //gzmsg << _pose.Pos()[0] << " " << _pose.Pos()[1] << " " << _pose.Pos()[2] << std::endl;
  //gzmsg << v[0] << " " << v[1] << " " << v[2] << std::endl;
  //gzmsg << v.Length() << std::endl;
  return angle;
}


//////////////////////////////////////////////////
ignition::math::Pose3d gazebo::sensors::RadiationSensor::GetPose() const
{
  return this->entity->GetWorldPose().Ign();
}

void gazebo::sensors::RadiationSensor::AddSource(RadiationSource *_rs)
{
  if ( this->sensor_type == ""){
    gzmsg << "adding source " <<_rs->GetSDF()->GetElement("topic")->Get<std::string>()<< std::endl;

    this->sources.push_back(_rs);
  }
  else if ( _rs->radiation_type == this->sensor_type){
    gzmsg << "adding source " <<_rs->GetSDF()->GetElement("topic")->Get<std::string>()<< std::endl;

    this->sources.push_back(_rs);
  }
}

void gazebo::sensors::RadiationSensor::RemoveSource(std::string source_name){
  for (int i = 0;i < this->sources.size();i++){
    if (this->sources[i]->name == source_name){
      gzmsg << this->sources[i]->name << " removed " << this->sources.size() << std::endl;
      this->sources.erase(this->sources.begin()+i);
      i--;
    }
  }
}

sdf::ElementPtr gazebo::sensors::RadiationSource::GetSDF(){
  return this->sdf;
}

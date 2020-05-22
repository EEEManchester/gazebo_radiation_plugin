#include <gazebo_radiation_plugins/RadiationSource.h>


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
    std::bind(&gazebo::sensors::RadiationSource::UpdateImpl, this,true));


  if (this->sdf->GetElement("topic"))
  {
    this->scanPub_pose = this->node->Advertise<msgs::Pose>(this->sdf->GetElement("topic")->Get<std::string>()+"/pose");
    this->scanPub_value = this->node->Advertise<msgs::Any>(this->sdf->GetElement("topic")->Get<std::string>()+"/value");}
    
    this->topic = this->sdf->GetElement("topic")->Get<std::string>();
    this->radiation_type = this->topic.substr(0, this->topic.find("/"));

    gzmsg<< this->radiation_type << std::endl;

  this->entity = this->world->GetEntity(this->ParentName());

  /*
  // Add the tag to all the RFID sensors.
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter)
  {
    if ((*iter)->Type() == "rfid")
    {
      std::dynamic_pointer_cast<RFIDSensor>(*iter)->AddTag(this);
    }
  }
  */
}

/*
void gazebo::sensors::RadiationSource::Load(const std::string &_worldName, sdf::ElementPtr _sdf)
{
  Sensor::Load(_worldName, _sdf);

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    std::bind(&gazebo::sensors::RadiationSource::OnUpdate, this));

  this->entity = this->world->GetEntity(this->ParentName());


  if (this->sdf->GetElement("topic"))
  {


    this->scanPub_pose = this->node->Advertise<msgs::Pose>(
        this->sdf->GetElement("topic")->Get<std::string>()+"_pose");
    this->scanPub_value = this->node->Advertise<msgs::Vector2d>(
        this->sdf->GetElement("topic")->Get<std::string>()+"_value");
  
    std::string pose_topic = this->sdf->GetElement("topic")->Get<std::string>()+"_pose";
    std::string rad_topic = this->sdf->GetElement("topic")->Get<std::string>()+"_value";
  }

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "radiation_sim_rosnode",
        ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("radiation_sim_rosnode"));


  

  gzmsg << "Example custom sensor loaded" << std::endl;


}
*/

void gazebo::sensors::RadiationSource::Fini()
{
  Sensor::Fini();
  this->entity.reset();
  this->radiation = 50.0;
}

void gazebo::sensors::RadiationSource::Init()
{
  Sensor::Init();

  this->radiation = 100.0;

}

bool gazebo::sensors::RadiationSource::UpdateImpl(const bool force)
    {

    msgs::Pose msg_pose;
    this->pose = this->GetPose();

    msgs::Set(&msg_pose, pose);

    msgs::Any msg_value;

    msg_value = msgs::ConvertAny(this->radiation);


    this->scanPub_pose->Publish(msg_pose);
    this->scanPub_value->Publish(msg_value);   

    }


ignition::math::Pose3d gazebo::sensors::RadiationSource::GetPose() const
{
  return this->entity->GetWorldPose().Ign();
}

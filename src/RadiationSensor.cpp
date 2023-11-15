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
      std::bind(&gazebo::sensors::RadiationSensor::UpdateImpl, this, true));

  if (this->sdf->GetElement("topic"))
  {
    this->radPub_pose = this->node->Advertise<msgs::Pose>(this->sdf->GetElement("topic")->Get<std::string>() + "/pose");
    this->radPub_value = this->node->Advertise<msgs::Any>(this->sdf->GetElement("topic")->Get<std::string>() + "/value");
  }

  this->topic = this->sdf->GetElement("topic")->Get<std::string>();

  //this->sensor_type = this->sdf->GetElement("sensor_type")->Get<std::string>();
  this->sensor_type = "";

  this->entity = this->world->EntityByName(this->ParentName());

  if (n.hasParam("sensors/" + this->topic + "/type"))
  {
    n.getParam("sensors/" + this->topic + "/type", this->sensor_type);
  }

  if (n.hasParam("sensors/" + this->topic + "/range"))
  {
    n.getParam("sensors/" + this->topic + "/range", this->sensor_range);
  }

  if (n.hasParam("sensors/" + this->topic + "/collimated"))
  {
    n.getParam("sensors/" + this->topic + "/collimated", this->collimated);
  }

  if (n.hasParam("sensors/" + this->topic + "/collimation_attenuation"))
  {
    n.getParam("sensors/" + this->topic + "/collimation_attenuation", this->collimation_attenuation);
  }

  if (n.hasParam("sensors/" + this->topic + "/poisson"))
  {
    n.getParam("sensors/" + this->topic + "/poisson", this->poisson);
  }

  if (n.hasParam("sensors/" + this->topic + "/sensitivity_function"))
  {
    n.getParam("sensors/" + this->topic + "/sensitivity_function", this->sensitivity_func);
  }

  if (n.hasParam("sensors/" + this->topic + "/mu"))
  {
    n.getParam("sensors/" + this->topic + "/mu", this->mu);
  }

  if (n.hasParam("sensors/" + this->topic + "/sigma"))
  {
    n.getParam("sensors/" + this->topic + "/sigma", this->sig);
  }

  if (n.hasParam("sensors/" + this->topic + "/angle_limit"))
  {
    n.getParam("sensors/" + this->topic + "/angle_limit", this->angle_limit);
  }
  if (n.hasParam("sensors/" + this->topic + "/azimuth_limit"))
  {
    n.getParam("sensors/" + this->topic + "/azimuth_limit", this->az_limit);
  }
  if (n.hasParam("sensors/" + this->topic + "/elevation_limit"))
  {
    n.getParam("sensors/" + this->topic + "/elevation_limit", this->el_limit);
  }

  if (n.hasParam("/attenuation_factors"))
  {
    n.getParam("/attenuation_factors", this->attenuation_factors);
  }

  
  frustum_points.push_back(std::pair<float,float>(-1*this->az_limit,-1*this->el_limit));
  frustum_points.push_back(std::pair<float,float>(this->az_limit,-1*this->el_limit));
  frustum_points.push_back(std::pair<float,float>(-1*this->az_limit,this->el_limit));
  frustum_points.push_back(std::pair<float,float>(this->az_limit,this->el_limit));

  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator iter = sensors.begin(); iter != sensors.end(); ++iter)
  {
    if ((*iter)->Type() == "radiation_source")
    {

      auto sensorPtr = std::static_pointer_cast<RadiationSource>(*iter).get();
      this->AddSource(sensorPtr);
    }
  }

  this->blockingRay = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(
      this->world->Physics()->CreateShape("ray", gazebo::physics::CollisionPtr()));
}

void gazebo::sensors::RadiationSensor::Fini()
{
  {
    boost::recursive_mutex::scoped_lock lock(*(
        this->world->Physics()->GetPhysicsUpdateMutex()));

    this->entity.reset();
    Sensor::Fini();
  }
}

void gazebo::sensors::RadiationSensor::Init()
{
  Sensor::Init();
}

bool gazebo::sensors::RadiationSensor::UpdateImpl(const bool force)
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

  if (this->gotSensor)
  {
    {
      boost::recursive_mutex::scoped_lock lock(*(
          this->world->Physics()->GetPhysicsUpdateMutex()));

      msgs::Pose msg_pose;
      this->pose = this->GetPose();
      msgs::Set(&msg_pose, pose);

      this->EvaluateSources();
      msgs::Any msg_value;
      msg_value = msgs::ConvertAny(this->radiation);

      this->radPub_pose->Publish(msg_pose);
      this->radPub_value->Publish(msg_value);
    }
  }
  return true;
}

//////////////////////////////////////////////////
void gazebo::sensors::RadiationSensor::EvaluateSources()
{

  std::vector<RadiationSource *>::const_iterator ci;

  // iterate through the tags contained given rfid tag manager

  double rad = 0.0;
  for (ci = this->sources.begin(); ci != this->sources.end(); ci++)
  {
    ignition::math::Pose3d pos = (*ci)->GetPose();
    double dist = this->CheckSourceRange(pos);

    if (dist <= this->sensor_range)
    {
      std::vector<raySegment> raySegments = CheckSourceViewable(this->GetPose().Pos(), pos.Pos(), (*ci)->name);
      std::string type = (*ci)->radiation_type;
      double value = (*ci)->radiation;

      double sensitivity;
      double angle = this->CheckSourceAngle(pos);
      if (this->sensitivity_func != "")
      {
        sensitivity = sensitivity_function(angle);
      }
      else
      {
        sensitivity = 1.0;
      }
      float within_angle_limit = 1.0;
      if (this->collimated != "")
      {
        within_angle_limit = CheckCollimation(angle, pos);
      }
      float within_range_limit = 1.0;
      if (fabs(this->sensor_range) <= fabs(dist))
      {
        within_range_limit = 0.0;
      }

      if (raySegments.empty())
      {
        rad += (within_range_limit * within_angle_limit * sensitivity * value * SolidAngle(dist));
      }
      else
      {
        rad += (within_range_limit * within_angle_limit * sensitivity * value * SolidAngle(dist) * AttenuationFactor(raySegments));
      }

      //gzmsg << within_range_limit << "  " << within_angle_limit << "  " << sensitivity << "  " << value << "  " << SolidAngle(dist) << AttenuationFactor(raySegments) << std::endl;
      //gzmsg << "radiation =  " << rad << std::endl;
    }
  }

  if (rad > 0.0)
  {
    // Discretise into integer events, with variance of a Poisson distribution (if user selected)
    if (this->poisson == true)
    {
      std::poisson_distribution<int> distribution(rad);
      int number = distribution(this->generator);
      this->radiation = number;
    }
    else
    {
      this->radiation = rad;
    }
  }
  else
  {
    this->radiation = rad;
  }
}
double gazebo::sensors::RadiationSensor::CheckCollimation(double angle, const ignition::math::Pose3d &_pose)
{
  if (this->collimated == "slot")
  {


    ignition::math::Vector3d v = _pose.Pos() - this->GetPose().Pos();
    ignition::math::Vector3d vx(1, 0, 0);
    //ignition::math::Vector3d vy(0, 1, 0);
    ignition::math::Vector3d v0 = this->GetPose().Rot().RotateVector(vx);
    //ignition::math::Vector3d v1 = this->GetPose().Rot().RotateVector(vy);

    v.Normalize();
    v0.Normalize();

    //gzmsg << "v " << v.X() << " "<< v.Y() << " "<< v.Z() << " " << std::endl;
    //gzmsg << "v0 " << v0.X() << " "<< v0.Y() << " "<< v0.Z() << " " << std::endl;

    ignition::math::Quaterniond q;
    ignition::math::Vector3d cross = v0.Cross(v);
    q.X() = cross.X();
    q.Y() = cross.Y();
    q.Z() = cross.Z();
    q.W() = sqrt(pow(v.Length(),2)*pow(v0.Length(),2)) + v0.Dot(v);
    q.Normalize();
    
    //gzmsg << "Q" << std::endl;
    //gzmsg << q.X() << "," << q.Y() << "," << q.Z() << "," << q.W() << std::endl;
    //gzmsg << q.Roll() << "," << q.Pitch() << "," << q.Yaw() << std::endl;
  
    ignition::math::Quaterniond q1;
    ignition::math::Vector3d vq(0.0,q.Pitch(),q.Yaw());
    q1 = q1.EulerToQuaternion(vq);


    float az = q.Yaw();
    float el = q.Pitch();

    float roll = this->GetPose().Rot().Roll();

    gzmsg << az << " " << el << " " << roll << std::endl;

    std::vector<std::pair<float,float> > rotated_frustum_points;
    
    for (int i =0;i<this->frustum_points.size();i++){
      std::pair<float,float> rot_pair(this->frustum_points[i].first*std::cos(roll)-this->frustum_points[i].second*std::sin(roll),this->frustum_points[i].first*std::sin(roll)+this->frustum_points[i].second*std::cos(roll));
      rotated_frustum_points.push_back(rot_pair);
      gzmsg << rot_pair.first <<"," << rot_pair.second << std::endl;
    }


    std::vector<std::pair<float,float> > bounding_lines;
    bounding_lines.push_back(calc_equations(rotated_frustum_points[0],rotated_frustum_points[1]));
    bounding_lines.push_back(calc_equations(rotated_frustum_points[2],rotated_frustum_points[3]));
    bounding_lines.push_back(calc_equations(rotated_frustum_points[0],rotated_frustum_points[2]));
    bounding_lines.push_back(calc_equations(rotated_frustum_points[1],rotated_frustum_points[3]));



    std::vector<float> values;
    for (int i = 0; i < bounding_lines.size();i++){
      values.push_back(bounding_lines[i].first*az + bounding_lines[i].second - el);
      gzmsg << bounding_lines[i].first << " "  << bounding_lines[i].second << std::endl;
    }


    gzmsg << values[0] << " " << values[1] << " "<< values[2] << " "<< values[3] << std::endl;

    if (((values[0] <=0)&(values[1] >=0))|((values[1] <=0)&(values[0] >=0))){
        if (((values[2] <=0)&(values[3] >=0))|((values[3] <=0)&(values[2] >=0))){
          gzwarn << "INSIDE FRUSTRUM" << std::endl;

          return 1.0;
    
      }
    }
    return this->collimation_attenuation;

  }
  else if (this->collimated == "pinhole")
  {
    if (fabs(this->angle_limit) <= fabs(angle))
    {
      return this->collimation_attenuation;
    }
    else
    {
      return 1.0;
    }
  }
  gzwarn << "COLLIMATION NOT IMPLEMENTED DEFAULTING TO UNCOLLIMATED" << std::endl;
  return 1.0;
}


std::pair<float,float>  gazebo::sensors::RadiationSensor::calc_equations(std::pair<float,float> i,std::pair<float,float> j){


    float grad; 
    float intercept;
    if (i.first != j.first) {
      grad = (i.second - j.second)/( i.first - j.first );
      intercept = i.second - grad*i.first;
    } else{
      grad = std::numeric_limits<int>::max();
      intercept = i.second - grad*i.first;
    }

    return std::pair<float,float>(grad,intercept);

}

double gazebo::sensors::RadiationSensor::SolidAngle(double dist)
{
  // Based on circular cross section detector - always normal to source
  double detector_radius = 1E-3;
  double correction_factor = 0.5 * (1 - (1 / std::sqrt(1 + pow(detector_radius, 2.0)))); // Correct intensity to 1 m value
  return (1 / correction_factor) * 0.5 * (1 - (dist / std::sqrt(pow(dist, 2.0) + pow(detector_radius, 2.0))));
}

double gazebo::sensors::RadiationSensor::AttenuationFactor(std::vector<raySegment> ray_vector)
{

  //gzmsg << "ray interations :" << std::endl;
  double attenuation_factor = 1.0;
  double material_attenuation = 0.0;
  XmlRpc::XmlRpcValue y;

  for (int i = 0; i < ray_vector.size(); i++)
  {
    //gzmsg << "transition from " << ray_vector[i].from << " to " << ray_vector[i].to << " at length: " << ray_vector[i].length << " ATTENUAITING " << attenuation_factor << std::endl;

    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = this->attenuation_factors.begin();
         it != this->attenuation_factors.end(); ++it)
    {
      XmlRpc::XmlRpcValue x = it->first;
      if (ray_vector[i].from.find(std::string(x)) != std::string::npos)
      {
        y = it->second;
        if (static_cast<double>(y) < 0.0)
        {
          material_attenuation = 0;
          attenuation_factor = 0.0;
        }
        else
        {
          material_attenuation = static_cast<double>(y);
        } //break;
      }
    }

    //old fake attenuation factor
    //attenuation_factor *= 1.0 - (ray_vector[i].length*material_attenuation);
    //andys attenuation factor
    attenuation_factor *= exp(-material_attenuation * ray_vector[i].length);
  }

  /*
  Add function here to do attenuation along the line!!!
  */
  return attenuation_factor;
}

double gazebo::sensors::RadiationSensor::sensitivity_function(double x)
{
  if (this->sensitivity_func == "gaussian")
  {
    return exp(-pow(x - this->mu, 2.0) / (2.0 * pow(this->sig, 2.0)));
  }
  else
  {
    return 1.0;
  }
}

//////////////////////////////////////////////////
double gazebo::sensors::RadiationSensor::CheckSourceRange(const ignition::math::Pose3d &_pose)
{
  ignition::math::Vector3d v;
  v = _pose.Pos() - this->GetPose().Pos();

  return v.Length();
}
//////////////////////////////////////////////////
float dot(ignition::math::Vector3d a, ignition::math::Vector3d b) //calculates dot product of a and b
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

float mag(ignition::math::Vector3d a) //calculates magnitude of a
{
  return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

double gazebo::sensors::RadiationSensor::CheckSourceAngle(const ignition::math::Pose3d &_pose)
{
  // copy sensor vector pos into a temp var
  ignition::math::Vector3d v0;
  ignition::math::Vector3d v1;
  ignition::math::Vector3d v(1, 0, 0);

  v0 = _pose.Pos() - this->GetPose().Pos();
  v1 = this->GetPose().Rot().RotateVector(v);
  double angle = std::acos(dot(v0, v1) / (mag(v0) * mag(v1)));

  return angle;
}

//////////////////////////////////////////////////
ignition::math::Pose3d gazebo::sensors::RadiationSensor::GetPose() const
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

void gazebo::sensors::RadiationSensor::AddSource(RadiationSource *_rs)
{

  {
    boost::recursive_mutex::scoped_lock lock(*(
        this->world->Physics()->GetPhysicsUpdateMutex()));

    if (this->sensor_type == "")
    {
      gzmsg << "adding source " << _rs->GetSDF()->GetElement("topic")->Get<std::string>() << std::endl;

      this->sources.push_back(_rs);
    }
    else if (_rs->radiation_type == this->sensor_type)
    {
      gzmsg << "adding source " << _rs->GetSDF()->GetElement("topic")->Get<std::string>() << std::endl;

      this->sources.push_back(_rs);
    }
  }
}

void gazebo::sensors::RadiationSensor::RemoveSource(std::string source_name)
{
  {
    boost::recursive_mutex::scoped_lock lock(*(
        this->world->Physics()->GetPhysicsUpdateMutex()));

    for (int i = 0; i < this->sources.size(); i++)
    {
      if (this->sources[i]->name == source_name)
      {
        gzmsg << this->sources[i]->name << " removed " << this->sources.size() << std::endl;
        this->sources.erase(this->sources.begin() + i);
      }
    }
  }
}

std::vector<raySegment> gazebo::sensors::RadiationSensor::CheckSourceViewable(ignition::math::Vector3d sensor_pos, ignition::math::Vector3d source_pos, std::string name)
{

  std::vector<raySegment> v;

  while (1)
  {
    std::string entityName = "";
    double blocking_dist;
    this->blockingRay->SetPoints(sensor_pos, source_pos);
    this->blockingRay->GetIntersection(blocking_dist, entityName);
    if (entityName == "")
    {
      return v;
    }
    else if (blocking_dist > (sensor_pos - source_pos).Length())
    {
      return v;
    }
    else if (entityName.find(name) != std::string::npos)
    {
      return v;
    }
    else
    {
      //entityName = entityName.substr(0, entityName.find("::"));
      if (v.empty())
      {
        v.push_back(raySegment(blocking_dist, std::string("free_space"), entityName));
      }
      else if (v.back().to == entityName)
      {
        v.push_back(raySegment(blocking_dist, entityName, std::string("free_space")));
      }
      else if (v.back().to == std::string("free_space"))
      {
        v.push_back(raySegment(blocking_dist, std::string("free_space"), entityName));
      }
      else
      {
        v.push_back(raySegment(blocking_dist, v.back().to, entityName));
      }

      ignition::math::Vector3d v1 = (source_pos - sensor_pos).Normalize() * (blocking_dist + 0.0001);

      sensor_pos = v1 + sensor_pos;
    }
  }
}

sdf::ElementPtr gazebo::sensors::RadiationSource::GetSDF()
{
  return this->sdf;
}

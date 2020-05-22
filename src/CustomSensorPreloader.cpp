#include <gazebo_radiation_plugins/CustomSensorPreloader.h>

#include <pluginlib/class_loader.h>
#include <gazebo/sensors/Sensor.hh>

#include <boost/filesystem.hpp>

#include <gazebo/physics/PhysicsFactory.hh>
#include <gazebo/sensors/SensorManager.hh>

// ignition-common is find_packaged by gazebo
//#include <ignition/common/StringUtils.hh>

namespace fs = boost::filesystem;

namespace gazebo
{

gazebo::CustomSensorPreloader::CustomSensorPreloader() = default;
CustomSensorPreloader::~CustomSensorPreloader()
{
  if (this->deferredLoadThread)
  {
    this->deferredLoadThread->join();
  }
};


std::vector<std::string> Split(std::string str, std::string token){
    std::vector<std::string>result;
    while(str.size()){
        int index = str.find(token);
        if(index!=std::string::npos){
            result.push_back(str.substr(0,index));
            str = str.substr(index+token.size());
            if(str.size()==0)result.push_back(str);
        }else{
            result.push_back(str);
            str = "";
        }
    }
    return result;
}

void CustomSensorPreloader::Init()
{
  
  pluginlib::ClassLoader<gazebo::sensors::Sensor> loader(
      "gazebo_radiation_plugins", "gazebo::sensors::Sensor");

  const auto names = loader.getDeclaredClasses();
  for (const auto& name : names)
  {
    std::string type;
    if (name.rfind("sensors/", 0) == 0 )
    {
      const auto parts = Split(name, "/");
      if (parts.size() != 2 || parts[1].empty())
      {
        gzerr << "CustomSensorPreloader: Wrong 'name' attribute of custom "
              << "sensor. It should have the form 'sensors/sensor_type', got "
              << name << std::endl;
        continue;
      }
      type = parts[1];
    }
    else
    {
      if (name.find('/') != std::string::npos)
      {
        gzerr << "CustomSensorPreloader: Wrong 'name' attribute of custom "
              << "sensor. It should have the form 'sensors/sensor_type', got "
              << name << std::endl;
        continue;
      }
      else
      {
        gzwarn << "CustomSensorPreloader: Attribute 'name' of custom sensor '"
               << name << "' is missing the 'sensors/' prefix. This may lead "
               << "to name collisions. Consider adding the prefix."
               << std::endl;
        type = name;
      }
    }

    const auto classType = loader.getClassType(name);
    const auto classParts = Split(classType, ":");

    if (classParts.size() != 5)
    {
      gzerr << "CustomSensorPreloader: Attribute 'type' of custom sensor "
            << name << " should have the form 'gazebo::sensors::ClassName', "
            << "got '" << classType << "' instead." << std::endl;
      continue;
    }

    const auto classname = classParts.back();
    const auto fullname = loader.getClassLibraryPath(name);

    if (fullname.empty())
    {
      gzerr << "CustomSensorPreloader: Could not find path to the library of "
            << "custom sensor '" << name << "'." << std::endl;
      continue;
    }
    else if (!fs::exists(fullname))
    {
      gzerr << "CustomSensorPreloader: Library '" << fullname << "' of custom "
            << "sensor '" << name << "' does not exists." << std::endl;
      continue;
    }

    this->ProcessCustomSensor(type, classname, fullname);
  }

  for (const auto& libraryDir : this->libraryDirs)
  {
    gzmsg << "CustomSensorPreloader: Adding GAZEBO_PLUGIN_PATH " << libraryDir
          << std::endl;
    gazebo::common::SystemPaths::Instance()->AddPluginPaths(libraryDir);
  }

  // We want to run after SensorFactory::RegisterAll() to be sure the factory
  // is fully initialized. Therefore, we need to defer the preloading a bit
  // until it finishes. Unfortunately, there's no nice callback that would get
  // called when sensors are loaded but before world loading. So we piggyback
  // on PhysicsFactory, which is known to create the physics engines between
  // sensors::load() and sensors::init() calls (in gazebo.cc).
  this->deferredLoadThread.reset(new std::thread(
      std::bind(&CustomSensorPreloader::DeferredPreloadSensors, this)));
}

void CustomSensorPreloader::DeferredPreloadSensors()
{
  // as stated above, wait for physics engine initialization
  // but if it should take too long, give up the wait - it's very probable that
  // nothing bad will happen
  const size_t max_iterations = 2000000;
  size_t it = 0;
  while (!gazebo::physics::PhysicsFactory::IsRegistered("ode") && it < max_iterations)
  {
    usleep(1);
    ++it;
  }

  // call all the saved registration functions
  for (const auto& tuple : this->sensorsToRegister)
  {
    const auto registerFunc = std::get<0>(*tuple);
    const auto type = std::get<1>(*tuple);
    const auto classname = std::get<2>(*tuple);
    const auto fullname = std::get<3>(*tuple);

    // Call the registration function
    (*registerFunc)();

    std::vector<std::string> types;
    sensors::SensorManager::Instance()->GetSensorTypes(types);

    if (std::find(types.begin(), types.end(), type) == types.end())
    {
      gzwarn << "CustomSensorPreloader: Custom sensor " << classname
             << " from library " << fullname
             << " was preloaded, but it did not register a sensor of type "
             << type << std::endl;
    }
    else
    {
      gzmsg << "CustomSensorPreloader: Preloaded custom sensor " << classname
            << " from library " << fullname << std::endl;
    }
  }

  this->sensorsToRegister.clear();
}

void CustomSensorPreloader::ProcessCustomSensor(const std::string& _type,
                                                const std::string& _classname, const std::string& _fullname)
{

  const auto path = fs::path(_fullname);
  auto filename = path.filename().string();
  const auto libraryDir = path.parent_path();

  this->libraryDirs.insert(libraryDir.string());

  // this implementation is inspired by gazebo::common::Plugin<T>::Create()

#ifdef __APPLE__
  {
    // replace .so with .dylib
    size_t soSuffix = filename.rfind(".so");
    if (soSuffix != std::string::npos)
    {
      const std::string macSuffix(".dylib");
      filename.replace(soSuffix, macSuffix.length(), macSuffix);
    }
  }
#elif _WIN32
  {
    // replace .so with .dll
    size_t soSuffix = filename.rfind(".so");
    if (soSuffix != std::string::npos)
    {
      const std::string winSuffix(".dll");
      filename.replace(soSuffix, winSuffix.length(), winSuffix);
    }
    size_t libPrefix = filename.find("lib");
    if (libPrefix == 0)
    {
      // remove the lib prefix
      filename.erase(0, 3);
    }
  }
#endif  // ifdef __APPLE__

  const auto fullname = (libraryDir / fs::path(filename)).string();

  void *dlHandle = dlopen(fullname.c_str(), RTLD_LAZY|RTLD_GLOBAL);
  if (!dlHandle)
  {
    gzerr << "CustomSensorPreloader: Failed to load custom sensor library "
          << fullname << ": " << dlerror() << std::endl;
    return;
  }

  // The GZ_REGISTER_STATIC_SENSOR macro generates function RegisterClassName()
  const auto registerFuncName = "Register" + _classname;
  const auto registerFunc = (registerFuncType) dlsym(dlHandle, registerFuncName.c_str());

  if (!registerFunc)
  {
    gzerr << "CustomSensorPreloader: Failed to resolve registration function "
          << registerFuncName << ": " << dlerror() << std::endl
          << "Did you call GZ_REGISTER_STATIC_SENSOR(\"" << _type << "\", "
          << _classname << ") inside your sensor source file?" << std::endl;
    return;
  }

  // The actual registerFunc call has to be done in the deferred loading thread.
  this->sensorsToRegister.insert(std::make_shared<registerTuple>(
      std::make_tuple(registerFunc, _type, _classname, fullname)));
}

void CustomSensorPreloader::Load(int _argc, char **_argv)
{
  // Nothing to do here, everything happens in Init()
}


void CustomSensorPreloader::Reset()
{
  // nothing to do here, everything happens in Init()
}

}

GZ_REGISTER_SYSTEM_PLUGIN(gazebo::CustomSensorPreloader)
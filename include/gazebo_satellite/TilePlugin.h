#include <string>
#include <memory>
#include <fstream>
#include <vector>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include "tileloader.h"

namespace gazebo {

  class TilePlugin: public WorldPlugin {

    public:
      TilePlugin();

      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    private:
      physics::WorldPtr parent_;

      std::unique_ptr<TileLoader> loader_;

      void createVisual(sdf::ElementPtr link);

      void createScript(const std::string& path, const boost::filesystem::path& imgPath);
  };
}


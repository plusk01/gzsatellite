#include <string>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo {

  class TilePlugin: public WorldPlugin {

    public:
      TilePlugin();

      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    private:

      physics::WorldPtr parent_;

      void createVisual(sdf::ElementPtr link);

  };
}


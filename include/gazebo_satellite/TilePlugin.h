#include <string>
#include <sstream>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <ros/package.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include "modelcreator.h"

namespace gazebo {

  class TilePlugin: public WorldPlugin {
    public:
      TilePlugin();

      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    private:
      physics::WorldPtr parent_;
  };
}


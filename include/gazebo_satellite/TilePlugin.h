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

#include "tileloader.h"

namespace gazebo {

  class TilePlugin: public WorldPlugin {
    public:
      using MapTile = gzworld::TileLoader::MapTile;
      TilePlugin();

      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    private:
      physics::WorldPtr parent_;

      std::unique_ptr<gzworld::TileLoader> loader_;

      sdf::SDFPtr createModel(const std::string& name, double width, double height, std::vector<MapTile> tiles,
                boost::filesystem::path scripts_dir, boost::filesystem::path textures_dir);
      cv::Mat stitch(const std::vector<MapTile>& tiles);
      void createScript(const std::string& path, const boost::filesystem::path& imgPath);
  };
}


#pragma once

#include <string>
#include <sstream>
#include <memory>
#include <fstream>
#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>

#include "tileloader.h"

namespace gzsatellite {

  struct GeoParams
  {
    std::string tileserver;
    double lat, lon;
    double zoom;

    double width, height;
    double shift_x, shift_y;
  };

  class ModelCreator
  {
  public:
    // Given a set of geographic parameters and the root directory
    // for storing working files, instantiate a model creator obj
    ModelCreator(const GeoParams& params, const std::string& root);

    sdf::SDFPtr createModel(const std::string& name, unsigned int quality);

    void getOriginLatLon(double& lat, double& lon);
    
  private:
    // tile loader data
    std::unique_ptr<TileLoader> loader_;
    GeoParams geo_params_;
    std::vector<TileLoader::MapTile> tiles_;

    // relevant directory paths
    boost::filesystem::path materials_dir_;
    boost::filesystem::path textures_dir_;
    boost::filesystem::path scripts_dir_;

    // world image information
    boost::filesystem::path world_img_path_;
    boost::filesystem::path world_scr_path_;
    std::string model_name_;
    unsigned int jpg_quality_;

    void downloadTiles();
    void createWorldImage();
    void createWorldScript();
    cv::Mat stitchTiles();
    sdf::ElementPtr createCollision(double xpos, double ypos);
    sdf::ElementPtr createVisual(double xpos, double ypos);
  };

}
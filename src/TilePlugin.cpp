#include "gazebo_satellite/TilePlugin.h"

namespace fs = boost::filesystem;

namespace gazebo {

static const std::string root = "./gzsatellite/";

TilePlugin::TilePlugin() {}

// ----------------------------------------------------------------------------

void TilePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent_ = _parent;

  std::string service, name;
  double lat, lon, zoom;
  double quality;
  double width, height;
  double width_below, width_above;
  double height_below, height_above;

  ros::NodeHandle nh("/gzworld");
  // Geographic paramters
  nh.param<std::string>("tileserver", service, "http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}");
  nh.param<double>("latitude", lat, 40.267463);
  nh.param<double>("longitude", lon, -111.635655);
  nh.param<double>("zoom", zoom, 22);
  // Geographic size parameters
  nh.param<double>("width", width, 50);
  nh.param<double>("height", height, 50);
  nh.param<double>("width_above", width_above, -1);
  nh.param<double>("width_below", width_below, -1);
  nh.param<double>("height_above", height_above, -1);
  nh.param<double>("height_below", height_below, -1);
  // Model parameters
  nh.param<std::string>("name", name, "Rock Canyon Park");
  nh.param<double>("jpg_quality", quality, 60);

  //
  // Create the model creator with parameters
  //

  width = 300;
  height = 300;

  gzworld::GeoParams params;
  params.tileserver   = service;
  params.lat          = lat;
  params.lon          = lon;
  params.zoom         = zoom;
  params.width        = width;
  params.height       = height;
  params.width_below  = width_below;
  params.width_above  = width_above;
  params.height_below = height_below;
  params.height_above = height_above;

  gzworld::ModelCreator m(params, root);

  //
  // Create a world model and add it to the Gazebo World
  //

  auto modelSDF = m.createModel(name, quality);
  this->parent_->InsertModelSDF(*modelSDF);
}

// ----------------------------------------------------------------------------

GZ_REGISTER_WORLD_PLUGIN(TilePlugin)
}
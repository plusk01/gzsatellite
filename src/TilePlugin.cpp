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
  double shift_x, shift_y;

  ros::NodeHandle nh("/gzworld");
  // Geographic paramters
  nh.param<std::string>("tileserver", service, "http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}");
  nh.param<double>("latitude", lat, 40.267463);
  nh.param<double>("longitude", lon, -111.635655);
  nh.param<double>("zoom", zoom, 22);
  // Geographic size parameters
  nh.param<double>("width", width, 50);
  nh.param<double>("height", height, 50);
  nh.param<double>("shift_ew", shift_x, 0);
  nh.param<double>("shift_ns", shift_y, 0);
  // Model parameters
  nh.param<std::string>("name", name, "Rock Canyon Park");
  nh.param<double>("jpg_quality", quality, 60);

  //
  // Create the model creator with parameters
  //

  shift_y = -0.225;
  width = 300;
  height = 300;

  gzworld::GeoParams params;
  params.tileserver   = service;
  params.lat          = lat;
  params.lon          = lon;
  params.zoom         = zoom;
  params.width        = width;
  params.height       = height;
  params.shift_x      = shift_x;
  params.shift_y      = shift_y;

  gzworld::ModelCreator m(params, root);

  //
  // Create a world model and add it to the Gazebo World
  //

  auto modelSDF = m.createModel(name, quality);
  this->parent_->InsertModelSDF(*modelSDF);

  gzmsg << "World model '" << name << "' (";
  gzmsg << std::setprecision(10) << lat << "," << lon << ") created." << std::endl;

  // std::cout << modelSDF->ToString() << std::endl;
}

// ----------------------------------------------------------------------------

GZ_REGISTER_WORLD_PLUGIN(TilePlugin)
}
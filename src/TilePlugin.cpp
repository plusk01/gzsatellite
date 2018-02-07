#include "gazebo_satellite/TilePlugin.h"

namespace fs = boost::filesystem;

namespace gazebo {

static const std::string root = "./gzsatellite/";

TilePlugin::TilePlugin() {}

// ----------------------------------------------------------------------------

void TilePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent_ = _parent;


  std::string object_uri = "http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}";
  double lat = 40.267463;
  double lon = -111.635655;
  double zoom = 3;
  unsigned int blocks = 2;

  loader_.reset(new TileLoader(root + "mapscache", object_uri, lat, lon, zoom, blocks));
  
  // blocking call to make all http requests for tile images
  auto tiles = loader_->loadTiles();



  // Create OGRE scripts directory
  fs::path dir(root + "materials");
  fs::create_directories(dir / "scripts");

  // Create a symlink to the current tile provider's cache directory
  fs::remove(dir / "textures");
  fs::create_symlink(loader_->cachePath(), dir / "textures");




  // Create a new, empty SDF model with a single link
  sdf::SDFPtr modelSDF(new sdf::SDF);
  sdf::init(modelSDF);
  sdf::ElementPtr model = modelSDF->Root()->AddElement("model");
  sdf::ElementPtr base_link = model->AddElement("link");

  createVisual(base_link);

  model->GetAttribute("name")->Set("parker_test");
  model->AddElement("static")->Set("true");

  this->parent_->InsertModelSDF(*modelSDF);
}

// ----------------------------------------------------------------------------

void TilePlugin::createVisual(sdf::ElementPtr link)
{

  //
  // Pose
  //

  msgs::Vector3d *position = new msgs::Vector3d();
  position->set_x(0);
  position->set_y(0);
  position->set_z(0);

  msgs::Quaternion *orientation = new msgs::Quaternion();
  orientation->set_w(1);

  msgs::Pose *pose = new msgs::Pose;
  pose->set_allocated_orientation(orientation);
  pose->set_allocated_position(position);

  ////////////////
  
  //
  // Geometry
  //

  msgs::Vector3d *normal = new msgs::Vector3d();
  normal->set_x(0);
  normal->set_y(0);
  normal->set_z(1);

  msgs::Vector2d *size = new msgs::Vector2d();
  size->set_x(300);
  size->set_y(300);

  msgs::PlaneGeom *plane = new msgs::PlaneGeom();
  plane->set_allocated_normal(normal);
  plane->set_allocated_size(size);

  msgs::Geometry *geo = new msgs::Geometry();
  geo->set_type(msgs::Geometry_Type_PLANE);
  geo->set_allocated_plane(plane);

  ///////////////

  //
  // Material
  //

  msgs::Material_Script *script = new msgs::Material_Script();
  std::string *uri1 = script->add_uri();
  *uri1 = "file:///home/plusk01/.ros/gzsatellite/materials/scripts/";
  std::string *uri2 = script->add_uri();
  *uri2 = "file:///home/plusk01/.ros/gzsatellite/materials/textures/";
  script->set_name("myimg");

  msgs::Material *material = new msgs::Material();
  material->set_allocated_script(script);

  //////////////

  msgs::Visual visual;
  visual.set_name("vtst");
  visual.set_allocated_geometry(geo);
  visual.set_allocated_pose(pose);
  visual.set_allocated_material(material);

  sdf::ElementPtr visualElem = msgs::VisualToSDF(visual);
  link->InsertElement(visualElem);
}

GZ_REGISTER_WORLD_PLUGIN(TilePlugin)
}
#include "gazebo_satellite/TilePlugin.h"

namespace gazebo {

TilePlugin::TilePlugin() {}

// ----------------------------------------------------------------------------

void TilePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent_ = _parent;
  // node = transport::NodePtr(new transport::Node());
  // node->Init(parent->GetName());
  // regen_sub = node->Subscribe("~/maze/regenerate", &TilePlugin::Regenerate, this);

  sdf::SDFPtr modelSDF(new sdf::SDF);
  sdf::init(modelSDF);
  sdf::ElementPtr model = modelSDF->Root()->AddElement("model");
  sdf::ElementPtr base_link = model->AddElement("link");

  createVisual(base_link);

  model->GetAttribute("name")->Set("parker_test");
  model->AddElement("static")->Set("true");

  this->parent_->InsertModelSDF(*modelSDF);


  const std::string package_path = ros::package::getPath("gazebo_satellite");
  gzmsg << package_path << std::endl;


  std::string object_uri = "http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}";
  double lat = 40.267463;
  double lon = -111.635655;
  double zoom = 3;
  unsigned int blocks = 2;
  loader_.reset(new TileLoader(object_uri, lat, lon, zoom, blocks));

  loader_->start();


  gzmsg << "Hi" << std::endl;
  gzmsg << boost::filesystem::current_path() << std::endl;
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
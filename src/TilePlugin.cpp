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
  // lat = 40.267363;
  // lon = -111.633664;
  double zoom = 22;
  unsigned int blocks = 15;

  //
  // Download Map Tiles
  //

  loader_.reset(new TileLoader(root + "mapscache", object_uri, lat, lon, zoom, blocks));
  
  gzmsg << "Downloading " << blocks << " blocks (" << blocks*blocks << ""
           " tiles) around (" << lat << ", " << lon << ")."
           " This may take a minute.\n";

  // blocking call to make all http requests for tile images
  auto tiles = loader_->loadTiles();

  gzmsg << "Done!\n";

  //
  // Set up directory structure for OGRE scripts
  //

  // Create OGRE scripts directory
  fs::path materials_dir(root + "materials");
  fs::path scripts_dir(materials_dir/"scripts");
  fs::create_directories(scripts_dir);

  // Create a symlink to the current tile provider's cache directory
  fs::path textures_dir(materials_dir/"textures");
  fs::remove(textures_dir); // TODO: should probably make sure its a symlink
  fs::create_symlink(loader_->cachePath(), textures_dir);

  //
  // Generate a terrain model using the tiles
  //

  // for (auto&& t : tiles) {
  //   std::cout << std::endl;
  //   std::cout << "Tile (" << t.x() << ", " << t.y() << ", " << t.z() << ")" << std::endl;
  //   std::cout << "\t" << t.imagePath() << std::endl;
  // }

  std::cout << loader_->resolution() << std::endl;






  for (auto&& t : tiles) {
    std::string materialName = t.imagePath().stem().string();
    std::string scriptPath = (scripts_dir/materialName).string() + ".material";

    // Create an OGRE script for this tile if it does not already exist
    if (!fs::exists(scriptPath))
      createScript(scriptPath, t.imagePath());
  }




  // Create a new, empty SDF model with a single link
  sdf::SDFPtr modelSDF(new sdf::SDF);
  sdf::init(modelSDF);
  sdf::ElementPtr model = modelSDF->Root()->AddElement("model");
  sdf::ElementPtr base_link = model->AddElement("link");



  for (auto&& t : tiles) {
    // NOTE(gareth): We invert the y-axis so that positive y corresponds
    // to north. We are in XYZ->ENU convention here.
    const int w = 256; //t.image().width();
    const int h = 256; //t.image().height();
    const double tile_w = w * loader_->resolution();
    const double tile_h = h * loader_->resolution();

    // std::cout << "tile: (" << tile_w << ", " << tile_h << ")" << std::endl;

    // Shift back such that (0, 0) corresponds to the exact latitude and
    // longitude the tile loader requested.
    // This is the local origin, in the frame of the map node.
    const double origin_x = loader_->originOffsetX() * tile_w;
    const double origin_y = loader_->originOffsetY() * tile_h;

    // std::cout << "Origin: (" << origin_x << ", " << origin_y << ")" << std::endl;

    // determine location of this tile, flipping y in the process
    // const double x = (t.x() - loader_->centerTileX()) * tile_w + origin_x;
    // const double y = -(t.y() - loader_->centerTileY()) * tile_h + origin_y;

    const double x =  (loader_->centerTileX() - t.x())*tile_w - tile_w/2.0 + origin_x;
    const double y = -(loader_->centerTileY() - t.y())*tile_h + tile_h/2.0 - origin_y;

    // std::cout << "loc: (" << x << ", " << y << ")" << std::endl;

    //  don't re-use any ids
    const std::string name_suffix =
        std::to_string(t.x()) + "_" + std::to_string(t.y()) + "_" +
        std::to_string(0) + "_" + std::to_string(0);



        //
        // Pose
        //

        msgs::Vector3d *position = new msgs::Vector3d();
        position->set_x(-x);
        position->set_y(-y);
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
        size->set_x(tile_w);
        size->set_y(tile_h);

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
        script->set_name(t.imagePath().stem().string());

        msgs::Material *material = new msgs::Material();
        material->set_allocated_script(script);

        //////////////

        msgs::Visual visual;
        visual.set_name(t.imagePath().stem().string());
        visual.set_allocated_geometry(geo);
        visual.set_allocated_pose(pose);
        visual.set_allocated_material(material);

        sdf::ElementPtr visualElem = msgs::VisualToSDF(visual);
        base_link->InsertElement(visualElem);



  }


  ///////////////////////////

  // createVisual(base_link);

  model->GetAttribute("name")->Set("parker_test");
  model->AddElement("static")->Set("true");

  this->parent_->InsertModelSDF(*modelSDF);
}

// ----------------------------------------------------------------------------

void TilePlugin::createScript(const std::string& path, const fs::path& imgPath)
{
  std::ofstream out(path);

  out << "material " << imgPath.stem().string()             << std::endl;
  out << "{"                                                << std::endl;
  out << "  receive_shadows false"                          << std::endl;
  // out << "  depth_bias -16 0"                               << std::endl;
  // out << "  cull_hardware none"                             << std::endl;
  // out << "  cull_software none"                             << std::endl;
  // out << "  depth_write off"                                << std::endl;
  // out << "  scene_blend alpha_blend"                        << std::endl;
  out << "  technique"                                      << std::endl;
  out << "  {"                                              << std::endl;
  out << "    lighting off"                                 << std::endl;
  out << "    pass"                                         << std::endl;
  out << "    {"                                            << std::endl;
  out << "      texture_unit"                               << std::endl;
  out << "      {"                                          << std::endl;
  out << "        texture " << imgPath.filename().string()  << std::endl;
  out << "        filtering bilinear"                       << std::endl;
  out << "      }"                                          << std::endl;
  out << "    }"                                            << std::endl;
  out << "  }"                                              << std::endl;
  out << "}";

  out.close();
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
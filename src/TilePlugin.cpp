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

  ros::NodeHandle nh("/gzworld");
  nh.param<std::string>("tileserver", service, "http://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}");
  nh.param<std::string>("name", name, "Rock Canyon Park");
  nh.param<double>("latitude", lat, 40.267463);
  nh.param<double>("longitude", lon, -111.635655);
  nh.param<double>("zoom", zoom, 22);

  //
  // Download Map Tiles
  //

  gzworld::TileLoader::WorldSize sizes;
  sizes.width = 300;
  sizes.height = 300;

  loader_.reset(new gzworld::TileLoader(root + "mapscache", service, lat, lon, zoom, sizes));

  // check if world model already exists
  std::string imageName = loader_->hash();

  
  gzmsg << "Downloading " << loader_->numTiles() << " tiles"
           " around (" << lat << ", " << lon << ")."
           " This may take a minute.\n";

  // blocking call to make all http requests for tile images
  auto tiles = loader_->loadTiles();

  gzmsg << "Done!\n";

  //
  // Set up directory structure for OGRE scripts
  //

  fs::path materials_dir(root + "materials");

  // Create OGRE scripts directory
  fs::path scripts_dir(materials_dir/"scripts");
  fs::create_directories(scripts_dir);

  // Create a textures dir for stitched map images
  fs::path textures_dir(materials_dir/"textures");
  fs::create_directories(textures_dir);

  //
  // Create the necessary components of the model
  //

  sdf::SDFPtr modelSDF = createModel(name, sizes.width, sizes.height, tiles, scripts_dir, textures_dir);

  this->parent_->InsertModelSDF(*modelSDF);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

sdf::SDFPtr TilePlugin::createModel(const std::string& name, double width, double height, std::vector<MapTile> tiles, fs::path scripts_dir, fs::path textures_dir)
{

  // Sort the tiles so that they are in an ordered
  // row major, matrix-like representation
  std::sort(tiles.begin(), tiles.end(),
    [](const MapTile& left, const MapTile& right) {
      return (left.y() < right.y()) || (left.y() == right.y() && left.x() < right.x());
    });

  //
  // Process the tiles and create one texture image
  //

  auto img = stitch(tiles);

  //
  // Save the image in the proper location
  //

  // get hashed image name using tiles for caching
  std::string imgName = loader_->hash();

  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(60);

  std::string imgPath = (textures_dir/imgName).string() + ".jpg";
  cv::imwrite(imgPath, img, compression_params);

  //
  // Create an OGRE script to go with this texture
  //

  std::string scriptPath = (scripts_dir/imgName).string() + ".material";

  // Create an OGRE script for this tile if it does not already exist
  if (!fs::exists(scriptPath))
    createScript(scriptPath, imgName+".jpg");


  //
  //
  //

  // Create a new, empty SDF model with a single link
  sdf::SDFPtr modelSDF(new sdf::SDF);
  sdf::init(modelSDF);
  sdf::ElementPtr model = modelSDF->Root()->AddElement("model");
  sdf::ElementPtr base_link = model->AddElement("link");


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
  size->set_x(width);
  size->set_y(height);

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
  *uri1 = "file://" + fs::absolute(scripts_dir).string();
  std::string *uri2 = script->add_uri();
  *uri2 = "file://" + fs::absolute(textures_dir).string();
  script->set_name(imgName);

  msgs::Material *material = new msgs::Material();
  material->set_allocated_script(script);

  //////////////

  msgs::Visual visual;
  visual.set_name(imgName);
  visual.set_allocated_geometry(geo);
  visual.set_allocated_pose(pose);
  visual.set_allocated_material(material);

  sdf::ElementPtr visualElem = msgs::VisualToSDF(visual);
  base_link->InsertElement(visualElem);


  ///////////////////////////

  model->GetAttribute("name")->Set(name);
  model->AddElement("static")->Set("true");

  return modelSDF;

}

// ----------------------------------------------------------------------------

cv::Mat TilePlugin::stitch(const std::vector<MapTile>& tiles)
{
  // find out how many tiles are in the x and y directions
  int cols, rows;
  int numTiles = loader_->numTiles(&cols, &rows);

  // Create an empty image with the proper dimensions
  int width = cols*loader_->imageSize();
  int height = rows*loader_->imageSize();
  cv::Mat result = cv::Mat::zeros(height, width, CV_8UC3);

  // Loop through each sorted tile and place appropriately in image
  for (int i=0; i<numTiles; i++)
  {
    cv::Mat tile = cv::imread(tiles[i].imagePath().string(), CV_LOAD_IMAGE_COLOR);

    // calculate image position
    const int tileCol = i%cols;
    const int tileRow = i/cols;
    const int startCol = tileCol*loader_->imageSize();
    const int startRow = tileRow*loader_->imageSize();

    // Create a region of interest into the result image
    cv::Mat masked(result, cv::Rect(startCol, startRow,
                              loader_->imageSize(), loader_->imageSize()));

    // Copy this tile into the result
    tile.copyTo(masked);
  }


  return result;
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

GZ_REGISTER_WORLD_PLUGIN(TilePlugin)
}
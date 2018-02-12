#include "gazebo_satellite/modelcreator.h"

namespace fs = boost::filesystem;

namespace gzworld {

ModelCreator::ModelCreator(const GeoParams& params, const std::string& root) :
  geo_params_(params)
{

  //
  // Create a new tile loader object
  //

  loader_.reset(new TileLoader(root+"/mapscache", params.tileserver,
                                params.lat, params.lon, params.zoom,
                                params.width, params.height));

  //
  // Setup proper directory structure
  //

  materials_dir_ = fs::absolute(root+"/materials");

  // Create OGRE scripts directory
  scripts_dir_ = materials_dir_/"scripts";
  fs::create_directories(scripts_dir_);

  // Create a textures dir for stitched map images
  textures_dir_ = materials_dir_/"textures";
  fs::create_directories(textures_dir_);

  //
  // Use the unique tileloader hash as the world image name
  //

  world_img_path_ = textures_dir_/(loader_->hash()+".jpg");
  world_scr_path_ = scripts_dir_/(loader_->hash()+".material");


  /*
    Directory structure:
    --------------------

    root
      mapscache
        (tiles from tileloader)
      materials
        scripts
          (generated scripts, one per stitched world)
        textures
          (generated world, stitched from tiles)
  */
}

// ----------------------------------------------------------------------------

sdf::SDFPtr ModelCreator::createModel(const std::string& name, unsigned int quality)
{
  // set model properties
  model_name_ = name;
  jpg_quality_ = quality;

  if (!fs::exists(world_img_path_))
    createWorldImage();

  // Now that the world image is created, we don't need to download any tiles,
  // but we do need the geographical information associated with each.
  if (tiles_.size() == 0)
    loader_->loadTiles(false);

  // If necessary, create the OGRE script associated with this world
  if (!fs::exists(world_scr_path_))
    createWorldScript();

  //
  // SDF Creation
  //

  // Create a new, empty SDF model with a single link
  sdf::SDFPtr modelSDF(new sdf::SDF);
  sdf::init(modelSDF);

  // Create a model element in the SDF
  sdf::ElementPtr model = modelSDF->Root()->AddElement("model");
  model->GetAttribute("name")->Set(name);
  model->AddElement("static")->Set("true");

  // Create the base link
  sdf::ElementPtr base_link = model->AddElement("link");

  double xpos = geo_params_.shift_x*geo_params_.width;
  double ypos = geo_params_.shift_y*geo_params_.height;

  sdf::ElementPtr collisionElem = createCollision(xpos, ypos);
  sdf::ElementPtr visualElem    = createVisual(xpos, ypos);

  base_link->InsertElement(collisionElem);
  base_link->InsertElement(visualElem);

  return modelSDF;
}

// ----------------------------------------------------------------------------

void ModelCreator::getOriginLatLon(double& lat, double& lon)
{
  // Convert percentage shift from center to meters from center
  double xpos = geo_params_.shift_x*geo_params_.width;
  double ypos = geo_params_.shift_y*geo_params_.height;

  // Size of square the tile in meters
  double tileSize = loader_->resolution()*loader_->imageSize();

  double x = loader_->centerTileX() + (xpos/tileSize);
  double y = loader_->centerTileY() + (ypos/tileSize);

  x += loader_->originOffsetX();
  y += loader_->originOffsetY();

  loader_->tileCoordsToLatLon(x, y, geo_params_.zoom, lat, lon);
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void ModelCreator::downloadTiles()
{
  // how many tiles are not cached and need to be downloaded?
  unsigned int num = loader_->numTilesToDownload();

  if (num > 0)
  {
    gzmsg << "Downloading " << num << " tiles"
             " around (" << geo_params_.lat << ", " << geo_params_.lon << ")."
             " This may take a minute." << std::endl;
  }

  // Download any necessary tiles
  tiles_ = loader_->loadTiles(true);

  if (num > 0)
    gzmsg << "Finished loading tiles" << std::endl;
}

// ----------------------------------------------------------------------------

void ModelCreator::createWorldImage()
{
  // Download (or use cached) tiles
  downloadTiles();

  gzmsg << "Stitching together " << loader_->numTiles() << " tiles...";

  // Stitch the indivisual tiles into one image
  auto img = stitchTiles();

  // Save the image to file
  std::vector<int> compression_params;
  compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
  compression_params.push_back(jpg_quality_);

  cv::imwrite(world_img_path_.string(), img, compression_params);

  gzmsg << "done." << std::endl;
}

// ----------------------------------------------------------------------------

cv::Mat ModelCreator::stitchTiles()
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
    cv::Mat tile = cv::imread(tiles_[i].imagePath().string(), CV_LOAD_IMAGE_COLOR);

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

void ModelCreator::createWorldScript()
{
  std::ofstream out(world_scr_path_.string());

  const std::string image_filename = world_img_path_.filename().string();
  const std::string image_name = world_img_path_.stem().string();

  out << "material " << image_name                          << std::endl;
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
  out << "        texture " << image_filename               << std::endl;
  out << "        filtering bilinear"                       << std::endl;
  out << "      }"                                          << std::endl;
  out << "    }"                                            << std::endl;
  out << "  }"                                              << std::endl;
  out << "}";

  out.close();
}

// ----------------------------------------------------------------------------

sdf::ElementPtr ModelCreator::createCollision(double xpos, double ypos)
{

  //
  // Pose
  //

  gazebo::msgs::Vector3d *position = new gazebo::msgs::Vector3d();
  position->set_x(xpos);
  position->set_y(ypos);
  position->set_z(0);

  gazebo::msgs::Quaternion *orientation = new gazebo::msgs::Quaternion();
  orientation->set_w(1);

  gazebo::msgs::Pose *pose = new gazebo::msgs::Pose;
  pose->set_allocated_orientation(orientation);
  pose->set_allocated_position(position);
  
  //
  // Geometry
  //

  gazebo::msgs::Vector3d *normal = new gazebo::msgs::Vector3d();
  normal->set_x(0);
  normal->set_y(0);
  normal->set_z(1);

  gazebo::msgs::Vector2d *size = new gazebo::msgs::Vector2d();
  size->set_x(geo_params_.width);
  size->set_y(geo_params_.height);

  gazebo::msgs::PlaneGeom *plane = new gazebo::msgs::PlaneGeom();
  plane->set_allocated_normal(normal);
  plane->set_allocated_size(size);

  gazebo::msgs::Geometry *geo = new gazebo::msgs::Geometry();
  geo->set_type(gazebo::msgs::Geometry_Type_PLANE);
  geo->set_allocated_plane(plane);

  //
  // Use the above pieces to create the collision element
  //

  gazebo::msgs::Collision collision;
  collision.set_name(world_img_path_.stem().string());
  collision.set_allocated_geometry(geo);
  collision.set_allocated_pose(pose);

  // Conver the collision msg to an element ptr
  return gazebo::msgs::CollisionToSDF(collision);
}

// ----------------------------------------------------------------------------

sdf::ElementPtr ModelCreator::createVisual(double xpos, double ypos)
{

  //
  // Pose
  //

  gazebo::msgs::Vector3d *position = new gazebo::msgs::Vector3d();
  position->set_x(xpos);
  position->set_y(ypos);
  position->set_z(0);

  gazebo::msgs::Quaternion *orientation = new gazebo::msgs::Quaternion();
  orientation->set_w(1);

  gazebo::msgs::Pose *pose = new gazebo::msgs::Pose;
  pose->set_allocated_orientation(orientation);
  pose->set_allocated_position(position);
  
  //
  // Geometry
  //

  gazebo::msgs::Vector3d *normal = new gazebo::msgs::Vector3d();
  normal->set_x(0);
  normal->set_y(0);
  normal->set_z(1);

  gazebo::msgs::Vector2d *size = new gazebo::msgs::Vector2d();
  size->set_x(geo_params_.width);
  size->set_y(geo_params_.height);

  gazebo::msgs::PlaneGeom *plane = new gazebo::msgs::PlaneGeom();
  plane->set_allocated_normal(normal);
  plane->set_allocated_size(size);

  gazebo::msgs::Geometry *geo = new gazebo::msgs::Geometry();
  geo->set_type(gazebo::msgs::Geometry_Type_PLANE);
  geo->set_allocated_plane(plane);

  //
  // Material
  //

  gazebo::msgs::Material_Script *script = new gazebo::msgs::Material_Script();
  std::string *uri1 = script->add_uri();
  *uri1 = "file://" + fs::absolute(scripts_dir_).string();
  std::string *uri2 = script->add_uri();
  *uri2 = "file://" + fs::absolute(textures_dir_).string();
  script->set_name(world_img_path_.stem().string());

  gazebo::msgs::Material *material = new gazebo::msgs::Material();
  material->set_allocated_script(script);

  //
  // Use the above pieces to create the visual element
  //

  gazebo::msgs::Visual visual;
  visual.set_name(world_img_path_.stem().string());
  visual.set_allocated_geometry(geo);
  visual.set_allocated_pose(pose);
  visual.set_allocated_material(material);

  // Conver the visual msg to an element ptr
  return gazebo::msgs::VisualToSDF(visual);
}

// ----------------------------------------------------------------------------

}
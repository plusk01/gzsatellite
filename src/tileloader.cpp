/**
 * TileLoader class for managing:
 *    - GPS to tile coordinate conversions
 *    - Tile image caching
 *    - Downloading images
 *
 * Based on the work of Gareth Cross, from the rviz_satellite package
 *    https://github.com/gareth-cross/rviz_satellite
 *
 * Parker Lusk, Feb 2018
 *
 ******************************************************************************
 * TileLoader.cpp
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *  Created on: 07/09/2014
 */

#include "gzsatellite/tileloader.h"

namespace gzsatellite {

namespace fs = boost::filesystem;

static size_t replaceRegex(const boost::regex &ex, std::string &str,
                           const std::string &replace)
{
  std::string::const_iterator start = str.begin(), end = str.end();
  boost::match_results<std::string::const_iterator> what;
  boost::match_flag_type flags = boost::match_default;
  size_t count = 0;
  while (boost::regex_search(start, end, what, ex, flags)) {
    str.replace(what.position(), what.length(), replace);
    start = what[0].second;
    count++;
  }
  return count;
}

// ----------------------------------------------------------------------------

TileLoader::TileLoader(const std::string& cacheRoot, const std::string& service,
                       double latitude, double longitude,
                       unsigned int zoom, double width, double height)
    : latitude_(latitude), longitude_(longitude), zoom_(zoom),
      width_(width), height_(height), object_uri_(service)
{

  //
  // Setup directory structure for downloaded images
  //

  // Hash the service URL so that tiles from different services are indepdendent
  std::hash<std::string> hash_fn;
  service_hash_ = std::to_string(hash_fn(object_uri_));

  // Create the directory structure for the tile images
  cache_path_ = fs::absolute(fs::path(cacheRoot + "/" + service_hash_));
  fs::create_directories(cache_path_);


  //
  // Calculate center tile coordinates
  //

  double x, y;
  latLonToTileCoords(latitude_, longitude_, zoom_, x, y);
  center_tile_x_ = std::floor(x);
  center_tile_y_ = std::floor(y);

  // fractional component
  origin_offset_x_ = x - center_tile_x_;
  origin_offset_y_ = y - center_tile_y_;

  // std::cout << "[DBG] center tile x: " << center_tile_x_ << std::endl;
  // std::cout << "[DBG] center tile y: " << center_tile_y_ << std::endl;

  // std::cout << "[DBG] origin offset x: " << origin_offset_x_ << std::endl;
  // std::cout << "[DBG] origin offset y: " << origin_offset_y_ << std::endl;

  //
  // Determine how many tiles around the center tile are needed
  //

  // Based on width/height, how many x block and y blocks?
  const double width_px = width / resolution();
  const double height_px = height / resolution();

  const double width_pct = width_px / imageSize();
  const double height_pct = height_px / imageSize();

  const double x_high_pct = origin_offset_x_ + width_pct/2;
  const double x_low_pct = origin_offset_x_ - width_pct/2;
  const double y_high_pct = origin_offset_y_ + height_pct/2;
  const double y_low_pct = origin_offset_y_ - height_pct/2;

  x_tiles_above_ =          std::floor(x_high_pct);
  x_tiles_below_ = std::abs(std::floor(x_low_pct));
  y_tiles_above_ =          std::floor(y_high_pct);
  y_tiles_below_ = std::abs(std::floor(y_low_pct));

  // std::cout << std::endl;
  // std::cout << "Resolution (m/px): " << resolution() << std::endl;
  // std::cout << "Width, Height (px): " << width_px << ", " << height_px << std::endl;
  // std::cout << "Width, Height (%): " << width_pct << ", " << height_pct << std::endl;
  // std::cout << "X: Low, High (%): " << x_low_pct << ", " << x_high_pct << std::endl;
  // std::cout << "Y: Low, High (%): " << y_low_pct << ", " << y_high_pct << std::endl;
  // std::cout << "X: Below, Above (tiles): " << x_tiles_below_ << ", " << x_tiles_above_ << std::endl;
  // std::cout << "Y: Below, Above (tiles): " << y_tiles_below_ << ", " << y_tiles_above_ << std::endl;
  // std::cout << std::endl;
}

// ----------------------------------------------------------------------------

const std::vector<TileLoader::MapTile>& TileLoader::loadTiles(bool download)
{
  // discard previous set of tiles and all pending requests
  abort();

  // determine what range of tiles we can load
  int min_x, max_x, min_y, max_y;
  tileRange(min_x, max_x, min_y, max_y);

  // initiate blocking requests
  for (int y = min_y; y <= max_y; y++) {
    for (int x = min_x; x <= max_x; x++) {
      // Generate filename
      const fs::path full_path = cachedPathForTile(x, y, zoom_);

      // Check if tile is already in the cache (or if we shouldn't download)
      if (fs::exists(full_path) || !download) {
        tiles_.push_back(MapTile(x, y, zoom_, full_path));

      } else {
        const std::string url = uriForTile(x, y);

        // send blocking request
        auto r = cpr::Get(url);

        // process the response
        if (r.status_code == 200) {
          // Save the response text (which is image data) as a binary
          std::fstream imgout(full_path.string(), std::ios::out | std::ios::binary);
          imgout.write(r.text.c_str(), r.text.size());
          imgout.close();

          // Let everyone know we have an image for this tile
          tiles_.push_back(MapTile(x, y, zoom_, full_path));

        } else {
          std::cerr << "Failed loading " << r.url << " with code " << r.status_code << std::endl;
        }
      }
    }
  }

  return tiles_;
}

// ----------------------------------------------------------------------------

bool TileLoader::insideCentreTile(double lat, double lon) const
{
  double x, y;
  latLonToTileCoords(lat, lon, zoom_, x, y);
  return (std::floor(x) == center_tile_x_ && std::floor(y) == center_tile_y_);
}

// ----------------------------------------------------------------------------

/// @see http://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
/// For explanation of these calculations.
void TileLoader::latLonToTileCoords(double lat, double lon, unsigned int zoom,
                                    double &x, double &y)
{
  if (zoom > 31) {
    throw std::invalid_argument("Zoom level " + std::to_string(zoom) + " too high");
  } else if (lat < -85.0511 || lat > 85.0511) {
    throw std::invalid_argument("Latitude " + std::to_string(lat) + " invalid");
  } else if (lon < -180 || lon > 180) {
    throw std::invalid_argument("Longitude " + std::to_string(lon) + " invalid");
  }

  const double rho = M_PI / 180;
  const double lat_rad = lat * rho;

  unsigned int n = (1 << zoom);
  x = n * ((lon + 180) / 360.0);
  y = n * (1 - (std::log(std::tan(lat_rad) + 1 / std::cos(lat_rad)) / M_PI)) / 2;

  // gzdbg << "Center tile coords: " << x << ", " << y << std::endl;
}

// ----------------------------------------------------------------------------

// Returns lat/lon of NW corner. x+0.5,y+0.5 would return lat/lon of center
void TileLoader::tileCoordsToLatLon(double x, double y, unsigned int zoom,
                                    double& lat, double& lon)
{
  unsigned int n = (1 << zoom);
  lon = x / n * 360.0 - 180.0;

  const double a = M_PI - 2.0 * M_PI * y / n;
  lat = 180.0 / M_PI * atan(0.5 * (exp(a) - exp(-a)));
}

// ----------------------------------------------------------------------------

double TileLoader::resolution() const
{
  return zoomToResolution(latitude_, zoom_);
}

// ----------------------------------------------------------------------------

double TileLoader::zoomToResolution(double lat, unsigned int zoom)
{
  const double lat_rad = lat * M_PI / 180;
  return 156543.034 * std::cos(lat_rad) / (1 << zoom);
}

// ----------------------------------------------------------------------------

void TileLoader::abort()
{
  tiles_.clear();
}

// ----------------------------------------------------------------------------

const int TileLoader::numTilesToDownload() const
{
  // determine what range of tiles we can load
  int min_x, max_x, min_y, max_y;
  tileRange(min_x, max_x, min_y, max_y);

  // Simply count how many tiles don't have an image on file
  unsigned int n = 0;
  for (int y = min_y; y <= max_y; y++)
    for (int x = min_x; x <= max_x; x++)
      if (!fs::exists(cachedPathForTile(x, y, zoom_)))
        n++;

  return n;
}

// ----------------------------------------------------------------------------

const int TileLoader::numTiles(int* x, int* y) const
{
  const int xx = x_tiles_above_ + x_tiles_below_ + 1;
  const int yy = y_tiles_above_ + y_tiles_below_ + 1;

  if (x != nullptr) *x = xx;
  if (y != nullptr) *y = yy;

  return xx*yy;
}

// ----------------------------------------------------------------------------

const std::string TileLoader::hash() const
{
  std::ostringstream os;

  // The service URI makes it unique
  os << serviceHash();

  // geographic info
  os << latitude_ << longitude_ << zoom_;

  // size information
  os << width_ << height_;

  std::hash<std::string> hash_fn;
  return std::to_string(hash_fn(os.str()));
}

// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

std::string TileLoader::uriForTile(int x, int y) const
{
  std::string object = object_uri_;
  //  place {x},{y},{z} with appropriate values
  replaceRegex(boost::regex("\\{x\\}", boost::regex::icase), object,
               std::to_string(x));
  replaceRegex(boost::regex("\\{y\\}", boost::regex::icase), object,
               std::to_string(y));
  replaceRegex(boost::regex("\\{z\\}", boost::regex::icase), object,
               std::to_string(zoom_));
  return object;
}

// ----------------------------------------------------------------------------

std::string TileLoader::cachedNameForTile(int x, int y, int z) const
{
  std::ostringstream os;
  os << "x" << x << "_y" << y << "_z" << z << ".jpg";
  return os.str();
}

// ----------------------------------------------------------------------------

fs::path TileLoader::cachedPathForTile(int x, int y, int z) const
{
  fs::path p = cache_path_ / cachedNameForTile(x, y, z);
  return p;
}

// ----------------------------------------------------------------------------

int TileLoader::maxTiles() const
{
  return (1 << zoom_) - 1;
}

// ----------------------------------------------------------------------------

void TileLoader::tileRange(int& min_x, int& max_x, int& min_y, int& max_y) const
{
  // determine what range of tiles we can load
  min_x = std::max(0, center_tile_x_ - x_tiles_below_);
  min_y = std::max(0, center_tile_y_ - y_tiles_below_);
  max_x = std::min(maxTiles(), center_tile_x_ + x_tiles_above_);
  max_y = std::min(maxTiles(), center_tile_y_ + y_tiles_above_);
}

} // namespace gzsatellite
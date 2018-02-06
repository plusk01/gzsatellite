#include "gazebo_satellite/TilePlugin.h"

namespace gazebo {

void TilePlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
  // this->parent = _parent;
  // node = transport::NodePtr(new transport::Node());
  // node->Init(parent->GetName());
  // regen_sub = node->Subscribe("~/maze/regenerate", &TilePlugin::Regenerate, this);
}

GZ_REGISTER_WORLD_PLUGIN(TilePlugin)
}
/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include "rviz_common/geometry.hpp"
#include "rviz_common/load_resource.hpp"
#include "rviz_default_plugins/robot/arrow.hpp"
#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

#include "pose_tool.h"

namespace rviz_plugins
{

Pose3DTool::Pose3DTool()
  : rviz_common::Tool()
  , arrow_(NULL)
{
}

Pose3DTool::~Pose3DTool()
{
  delete arrow_;
}

void
Pose3DTool::onInitialize()
{
  arrow_ = new rviz_default_plugins::Arrow( scene_manager_, NULL );
  arrow_->setScale( Ogre::Vector3( 2.0f, 2.0f, 2.0f ));
}

int
Pose3DTool::processMouseEvent( rviz_common::ViewportMouseEvent& event )
{
  int flags = 0;
  Ogre::Vector3 pos;
  Ogre::Vector3 normal;
  if( !rviz_common::geometry::getPointOnPlaneFromWindowXY( event.viewport,
                                          Ogre::Plane( Ogre::Vector3::UNIT_Z, 0.0f ),
                                          event.x, event.y, pos, normal ))
  {
    return flags;
  }

  if( event.leftDown() )
  {
    // state_ = Position;
    // pos_ = pos;
  }

  if( event.type == QEvent::MouseButtonRelease && event.button == Qt::LeftButton )
  {
    // if( state_ == Orientation )
    // {
      double x = pos.x - 0; // Using 0 as reference point since we removed state_
      double y = pos.y - 0;
      double theta = atan2( y, x );
      onPoseSet( pos.x, pos.y, pos.z, theta );
      flags |= rviz_common::Render;
    // }
    // state_ = Height;
  }

  if( event.rightDown() )
  {
    // state_ = Position;
  }

  // Using direct positioning instead of state machine
  arrow_->setPosition( pos );
  arrow_->setOrientation( Ogre::Quaternion( Ogre::Degree(90), Ogre::Vector3::UNIT_X ));
  flags |= rviz_common::Render;

  if( event.type == QEvent::MouseMove && event.left() )
  {
    // state_ = Orientation;
  }

  return flags;
}
}

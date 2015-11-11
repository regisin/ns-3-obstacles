/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015 University of Nevada, Reno
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Paulo Regis <pregis@nevada.unr.edu>
 */
#ifndef RANDOM_WALK_3D_MOBILITY_MODEL_H
#define RANDOM_WALK_3D_MOBILITY_MODEL_H

#include "ns3/object.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/box.h"
#include "ns3/random-variable-stream.h"
#include "mobility-model.h"
#include "constant-velocity-helper.h"

namespace ns3 {


/**
 * \ingroup mobility
 * \brief 3D random walk mobility model.
 *
 * Each instance moves with a speed and direction choosen at random
 * with the user-provided random variables until
 * either a fixed distance has been walked or until a fixed amount
 * of time. If we hit one of the boundaries (specified by a rectangle),
 * of the model, we rebound on the boundary with a reflexive angle
 * and speed. This model is often identified as a brownian motion
 * model.
 */
class RandomWalk3dMobilityModel : public MobilityModel
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  /** An enum representing the different working modes of this module. */
  enum Mode  {
    MODE_DISTANCE,
    MODE_TIME
  };
  /**
   * \param obstacle an obstacle to be added
   *
   * The box must be inside the boundaries,
   * this method does not verify it.
   * This method assumes the box has dimensions > 0
   */
  void AddObstacle(const Box &obstacle);

private:
  /**
   * \brief Performs the rebound of the node if it reaches a boundary
   * \param timeLeft The remaining time of the walk
   */
  void Rebound (Time timeLeft);
  /**
   * Walk according to position and velocity, until distance is reached,
   * time is reached, or intersection with the bounding box
   */
  void DoWalk (Time timeLeft);
  /**
   * Perform initialization of the object before MobilityModel::DoInitialize ()
   */
  void DoInitializePrivate (void);
  virtual void DoDispose (void);
  virtual void DoInitialize (void);
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;
  virtual int64_t DoAssignStreams (int64_t);

  ConstantVelocityHelper m_helper; //!< helper for this object
  EventId m_event; //!< stored event ID 
  enum Mode m_mode; //!< whether in time or distance mode
  double m_modeDistance; //!< Change direction and speed after this distance
  Time m_modeTime; //!< Change current direction and speed after this delay
  Ptr<RandomVariableStream> m_speed; //!< rv for picking speed
  Ptr<RandomVariableStream> m_direction; //!< rv for picking direction
  Box m_bounds; //!< Bounds of the area to cruise

  Ptr<RandomVariableStream> m_pitch; //!< rv for picking pitch
  mutable std::vector<Box> m_obstacles; // list of obstacles
  int m_closestObstacle; // if collision is detected, this wil be set as the id of the obstacle in the array
};

} // namespace ns3

#endif /* RANDOM_WALK_3D_MOBILITY_MODEL_H */

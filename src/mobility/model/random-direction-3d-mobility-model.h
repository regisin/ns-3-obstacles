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
#ifndef RANDOM_DIRECTION_3D_MOBILITY_MODEL_H
#define RANDOM_DIRECTION_3D_MOBILITY_MODEL_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/nstime.h"
#include "ns3/event-id.h"
#include "ns3/box.h"
#include "ns3/random-variable-stream.h"
#include "mobility-model.h"
#include "constant-velocity-helper.h"

namespace ns3 {

/**
 * \ingroup mobility
 * \brief Random direction mobility model 3D.
 *
 * The movement of objects is based on random directions: each object
 * pauses for a specific delay, chooses a random direction and speed and 
 * then travels in the specific direction until it reaches one of
 * the boundaries of the model. When it reaches the boundary, it pauses,
 * selects a new direction and speed, aso.
 */
class RandomDirection3dMobilityModel : public MobilityModel
{
public:
  /**
   * Register this type with the TypeId system.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  RandomDirection3dMobilityModel ();
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
   * Set a new direction and speed
   */
  void ResetDirectionAndSpeed (void);
  /**
   * Pause, cancel currently scheduled event, schedule end of pause event
   */
  void BeginPause (void);
  /**
   * Set new velocity and direction, and schedule next pause event  
   * \param direction (radians)
   */
  void SetDirectionAndPitchAndSpeed (double direction, double pitch);
  /**
   * Sets a new random direction and calls SetDirectionAndSpeed
   */
  void DoInitializePrivate (void);
  virtual void DoDispose (void);
  virtual void DoInitialize (void);
  virtual Vector DoGetPosition (void) const;
  virtual void DoSetPosition (const Vector &position);
  virtual Vector DoGetVelocity (void) const;
  virtual int64_t DoAssignStreams (int64_t);

  Ptr<UniformRandomVariable> m_direction; //!< rv to control direction
  Ptr<RandomVariableStream> m_speed; //!< a random variable to control speed
  Ptr<RandomVariableStream> m_pause; //!< a random variable to control pause 
  EventId m_event; //!< event ID of next scheduled event
  ConstantVelocityHelper m_helper; //!< helper for velocity computations
  Box m_bounds; //!< Bounds of the area to cruise

  Ptr<UniformRandomVariable> m_pitch; //!< rv for picking pitch
  mutable std::vector<Box> m_obstacles; // list of obstacles
  int m_closestObstacle; // if collision is detected, this wil be set as the id of the obstacle in the array
};

} // namespace ns3

#endif /* RANDOM_DIRECTION_3D_MOBILITY_MODEL_H */

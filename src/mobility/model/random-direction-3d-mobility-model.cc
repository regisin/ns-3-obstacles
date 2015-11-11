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
#include "ns3/simulator.h"
#include <algorithm>
#include <cmath>
#include "ns3/log.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "random-direction-3d-mobility-model.h"

namespace ns3 {

  NS_LOG_COMPONENT_DEFINE ("RandomDirection3dMobilityModel");

  NS_OBJECT_ENSURE_REGISTERED (RandomDirection3dMobilityModel);


  TypeId
  RandomDirection3dMobilityModel::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::RandomDirection3dMobilityModel")
	    .SetParent<MobilityModel> ()
	    .SetGroupName ("Mobility")
	    .AddConstructor<RandomDirection3dMobilityModel> ()
	    .AddAttribute ("Bounds", "The 3d bounding box",BoxValue (Box (-100, 100, -100, 100, 0, 100)),MakeBoxAccessor (&RandomDirection3dMobilityModel::m_bounds),MakeBoxChecker ())
	    .AddAttribute ("Speed", "A random variable to control the speed (m/s).",StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=2.0]"),MakePointerAccessor (&RandomDirection3dMobilityModel::m_speed),MakePointerChecker<RandomVariableStream> ())
	    .AddAttribute ("Pause", "A random variable to control the pause (s).",StringValue ("ns3::ConstantRandomVariable[Constant=2.0]"),MakePointerAccessor (&RandomDirection3dMobilityModel::m_pause),MakePointerChecker<RandomVariableStream> ())
	    ;
    return tid;
  }

  RandomDirection3dMobilityModel::RandomDirection3dMobilityModel ()
  {
    m_direction = CreateObject <UniformRandomVariable> ();
    m_pitch = CreateObject <UniformRandomVariable> ();
    m_closestObstacle = -1;
  }

  void
  RandomDirection3dMobilityModel::AddObstacle (const Box &obstacle)
  {
    m_obstacles.push_back(obstacle);
  }

  void
  RandomDirection3dMobilityModel::DoDispose (void)
  {
    // chain up.
    MobilityModel::DoDispose ();
  }
  void
  RandomDirection3dMobilityModel::DoInitialize (void)
  {
    DoInitializePrivate ();
    MobilityModel::DoInitialize ();
  }

  void
  RandomDirection3dMobilityModel::DoInitializePrivate (void)
  {
    double direction = m_direction->GetValue (0, 2 * M_PI);
    double pitch = m_direction->GetValue (0, M_PI);
    SetDirectionAndPitchAndSpeed (direction, pitch);
  }

  void
  RandomDirection3dMobilityModel::BeginPause (void)
  {
    m_helper.Update ();
    m_helper.Pause ();
    Time pause = Seconds (m_pause->GetValue ());
    m_event.Cancel ();
    m_event = Simulator::Schedule (pause, &RandomDirection3dMobilityModel::ResetDirectionAndSpeed, this);
    NotifyCourseChange ();
  }

  void
  RandomDirection3dMobilityModel::SetDirectionAndPitchAndSpeed (double direction, double pitch)
  {
    NS_LOG_FUNCTION_NOARGS ();
    m_helper.UpdateWithBounds (m_bounds);
    Vector position = m_helper.GetCurrentPosition ();
    double speed = m_speed->GetValue ();
    const Vector vel (std::cos (direction) * std::sin (pitch) * speed,
		      std::sin (direction) * std::sin (pitch) * speed,
		      std::cos (pitch) * speed);
    m_helper.SetVelocity (vel);
    m_helper.Unpause ();
    Vector next = m_bounds.CalculateIntersection (position, vel);

    double distance = CalculateDistance (position, next);
    m_closestObstacle = -1;
    int obstacle_id = 0;
    for (std::vector<Box>::iterator it = m_obstacles.begin() ; it != m_obstacles.end(); ++it)
      {
	Vector collision;
	if (it->WillCollide(position,vel,collision) && CalculateDistance(position,collision) < distance)
	  {
	    distance = CalculateDistance(position,collision);
	    m_closestObstacle = obstacle_id;
	  }
	obstacle_id++;
      }

    Time delay = Seconds (distance / speed);
    m_event.Cancel ();
    m_event = Simulator::Schedule (delay, &RandomDirection3dMobilityModel::BeginPause, this);
    NotifyCourseChange ();
  }
  void
  RandomDirection3dMobilityModel::ResetDirectionAndSpeed (void)
  {
    double direction = m_direction->GetValue (0, 2*M_PI);
    double pitch = m_pitch->GetValue (0, M_PI);

    m_helper.UpdateWithBounds (m_bounds);
    Vector position = m_helper.GetCurrentPosition ();

    if (m_closestObstacle > -1) {
	switch (m_obstacles[m_closestObstacle].GetClosestSide(position))
	{
	  case Box::RIGHT:
	    direction += -M_PI / 2;
	    break;
	  case Box::LEFT:
	    direction += M_PI / 2;
	    break;
	  case Box::TOP:
	    direction += -M_PI;
	    break;
	  case Box::BOTTOM:
	    direction += M_PI;
	    break;
	  case Box::UP:
	    pitch += -M_PI / 2;
	    break;
	  case Box::DOWN:
	    pitch += M_PI / 2;
	    break;
	}
    }else{
	switch (m_bounds.GetClosestSide (position))
	{
	  case Box::RIGHT:
	    direction += M_PI / 2;
	    break;
	  case Box::LEFT:
	    direction += -M_PI / 2;
	    break;
	  case Box::TOP:
	    direction += M_PI;
	    break;
	  case Box::BOTTOM:
	    direction += -M_PI;
	    break;
	  case Box::UP:
	    pitch += M_PI / 2;
	    break;
	  case Box::DOWN:
	    pitch += -M_PI / 2;
	    break;
	}
    }

    SetDirectionAndPitchAndSpeed (direction, pitch);
  }
  Vector
  RandomDirection3dMobilityModel::DoGetPosition (void) const
  {
    m_helper.UpdateWithBounds (m_bounds);
    return m_helper.GetCurrentPosition ();
  }
  void
  RandomDirection3dMobilityModel::DoSetPosition (const Vector &position)
  {
    m_helper.SetPosition (position);
    Simulator::Remove (m_event);
    m_event.Cancel ();
    m_event = Simulator::ScheduleNow (&RandomDirection3dMobilityModel::DoInitializePrivate, this);
  }
  Vector
  RandomDirection3dMobilityModel::DoGetVelocity (void) const
  {
    return m_helper.GetVelocity ();
  }
  int64_t
  RandomDirection3dMobilityModel::DoAssignStreams (int64_t stream)
  {
    m_direction->SetStream (stream);
    m_speed->SetStream (stream + 1);
    m_pause->SetStream (stream + 2);
    m_pitch->SetStream (stream + 3);
    return 3;
  }

} // namespace ns3

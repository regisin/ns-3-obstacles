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
#include "random-walk-3d-mobility-model.h"
#include "ns3/enum.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include <cmath>

namespace ns3 {

  NS_LOG_COMPONENT_DEFINE ("RandomWalk3d");

  NS_OBJECT_ENSURE_REGISTERED (RandomWalk3dMobilityModel);

  TypeId
  RandomWalk3dMobilityModel::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::RandomWalk3dMobilityModel")
	    .SetParent<MobilityModel> ()
	    .SetGroupName ("Mobility")
	    .AddConstructor<RandomWalk3dMobilityModel> ()
	    .AddAttribute ("Bounds","Bounds of the area to cruise.",BoxValue (Box (0.0, 100.0, 0.0, 100.0, 0.0, 100.0)),MakeBoxAccessor (&RandomWalk3dMobilityModel::m_bounds),MakeBoxChecker ())
	    .AddAttribute ("Time","Change current direction and speed after moving for this delay.",TimeValue (Seconds (1.0)),MakeTimeAccessor (&RandomWalk3dMobilityModel::m_modeTime),MakeTimeChecker ())
	    .AddAttribute ("Distance","Change current direction and speed after moving for this distance.",DoubleValue (1.0),MakeDoubleAccessor (&RandomWalk3dMobilityModel::m_modeDistance),MakeDoubleChecker<double> ())
	    .AddAttribute ("Mode","The mode indicates the condition used to ""change the current speed and direction",EnumValue (RandomWalk3dMobilityModel::MODE_DISTANCE),MakeEnumAccessor (&RandomWalk3dMobilityModel::m_mode),MakeEnumChecker (RandomWalk3dMobilityModel::MODE_DISTANCE, "Distance",RandomWalk3dMobilityModel::MODE_TIME, "Time"))
	    .AddAttribute ("Direction","A random variable used to pick the direction (radians).",StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.283184]"),MakePointerAccessor (&RandomWalk3dMobilityModel::m_direction),MakePointerChecker<RandomVariableStream> ())
	    .AddAttribute ("Pitch","A random variable used to pick the pitch (radians).",StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=3.141592]"),MakePointerAccessor (&RandomWalk3dMobilityModel::m_pitch),MakePointerChecker<RandomVariableStream> ())
	    .AddAttribute ("Speed","A random variable used to pick the speed (m/s).",StringValue ("ns3::UniformRandomVariable[Min=2.0|Max=4.0]"),MakePointerAccessor (&RandomWalk3dMobilityModel::m_speed),MakePointerChecker<RandomVariableStream> ());
    return tid;
  }

  void
  RandomWalk3dMobilityModel::AddObstacle (const Box &obstacle)
  {
    m_obstacles.push_back(obstacle);
  }

  void
  RandomWalk3dMobilityModel::DoInitialize (void)
  {
    DoInitializePrivate ();
    MobilityModel::DoInitialize ();
  }

  void
  RandomWalk3dMobilityModel::DoInitializePrivate (void)
  {
    m_helper.Update ();
    double speed = m_speed->GetValue ();
    double direction = m_direction->GetValue ();
    double pitch = m_pitch->GetValue ();
    Vector vector (std::cos (direction) * std::sin(pitch) * speed,
		   std::sin (direction) * std::sin(pitch) * speed,
		   std::cos (pitch) * speed);
    m_helper.SetVelocity (vector);
    m_helper.Unpause ();

    Time delayLeft;
    if (m_mode == RandomWalk3dMobilityModel::MODE_TIME)
      {
	delayLeft = m_modeTime;
      }
    else
      {
	delayLeft = Seconds (m_modeDistance / speed);
      }
    DoWalk (delayLeft);
  }

  void
  RandomWalk3dMobilityModel::DoWalk (Time delayLeft)
  {
    Vector position = m_helper.GetCurrentPosition ();
    Vector speed = m_helper.GetVelocity ();

    double speedM = std::sqrt(speed.x*speed.x + speed.y*speed.y + speed.z*speed.z);

    Vector nextPosition = position;
    nextPosition.x += speed.x * delayLeft.GetSeconds ();
    nextPosition.y += speed.y * delayLeft.GetSeconds ();
    nextPosition.z += speed.z * delayLeft.GetSeconds ();

    m_event.Cancel ();

    if (m_bounds.IsInside (nextPosition))
      {
	double distance = CalculateDistance (position, nextPosition);
	m_closestObstacle = -1;
	int obstacle_id = 0;
	for (std::vector<Box>::iterator it = m_obstacles.begin() ; it != m_obstacles.end(); ++it)
	  {
	    Vector collision;
	    if (it->WillCollide(position,speed,collision) && CalculateDistance(position,collision) < distance)
	      {
		distance = CalculateDistance(position,collision);
		m_closestObstacle = obstacle_id;
	      }
	    obstacle_id++;
	  }
	Time delay_tmp = Seconds(distance/speedM);
	m_event = Simulator::Schedule (delay_tmp, &RandomWalk3dMobilityModel::DoInitializePrivate, this);
      }
    else
      {
	nextPosition = m_bounds.CalculateIntersection (position, speed);
	double distance = CalculateDistance (position, nextPosition);
	m_closestObstacle = -1;
	int obstacle_id = 0;
	for (std::vector<Box>::iterator it = m_obstacles.begin() ; it != m_obstacles.end(); ++it)
	  {
	    Vector collision;
	    if (it->WillCollide(position,speed,collision) && CalculateDistance(position,collision) < distance)
	      {
		distance = CalculateDistance(position,collision);
		m_closestObstacle = obstacle_id;
	      }
	    obstacle_id++;
	  }
	Time delay_tmp = Seconds(distance/speedM);

	m_event = Simulator::Schedule (delay_tmp, &RandomWalk3dMobilityModel::Rebound, this,
				       delayLeft - delay_tmp);
      }
    NotifyCourseChange ();
  }

  void
  RandomWalk3dMobilityModel::Rebound (Time delayLeft)
  {
    m_helper.UpdateWithBounds (m_bounds);
    Vector position = m_helper.GetCurrentPosition ();
    Vector speed = m_helper.GetVelocity ();
    Box bx;
    if (m_closestObstacle == -1){
	bx = m_bounds;
    }
    else
      {
	bx = m_obstacles[m_closestObstacle];
      }
    switch (bx.GetClosestSide (position))
    {
      case Box::RIGHT:
      case Box::LEFT:
	speed.x = -speed.x;
	break;
      case Box::TOP:
      case Box::BOTTOM:
	speed.y = -speed.y;
	break;
      case Box::UP:
      case Box::DOWN:
	speed.z = -speed.z;
	break;
    }
    m_helper.SetVelocity (speed);
    m_helper.Unpause ();
    DoWalk (delayLeft);
  }

  void
  RandomWalk3dMobilityModel::DoDispose (void)
  {
    // chain up
    MobilityModel::DoDispose ();
  }
  Vector
  RandomWalk3dMobilityModel::DoGetPosition (void) const
  {
    m_helper.UpdateWithBounds (m_bounds);
    return m_helper.GetCurrentPosition ();
  }
  void
  RandomWalk3dMobilityModel::DoSetPosition (const Vector &position)
  {
    NS_ASSERT (m_bounds.IsInside (position));
    m_helper.SetPosition (position);
    Simulator::Remove (m_event);
    m_event = Simulator::ScheduleNow (&RandomWalk3dMobilityModel::DoInitializePrivate, this);
  }
  Vector
  RandomWalk3dMobilityModel::DoGetVelocity (void) const
  {
    return m_helper.GetVelocity ();
  }
  int64_t
  RandomWalk3dMobilityModel::DoAssignStreams (int64_t stream)
  {
    m_speed->SetStream (stream);
    m_direction->SetStream (stream + 1);
    m_pitch->SetStream (stream + 2);
    return 3;
  }


} // namespace ns3

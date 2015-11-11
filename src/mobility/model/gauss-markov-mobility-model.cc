/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 Dan Broyles
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
 * Author: Dan Broyles <dbroyl01@ku.edu>
 * Modifications made by: Paulo Regis <pregis@nevada.unr.edu>
 */
#include <cmath>
#include "ns3/simulator.h"
#include "ns3/double.h"
#include "ns3/pointer.h"
#include "ns3/string.h"
#include "gauss-markov-mobility-model.h"
#include "position-allocator.h"

namespace ns3 {

  NS_OBJECT_ENSURE_REGISTERED (GaussMarkovMobilityModel);

  TypeId
  GaussMarkovMobilityModel::GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::GaussMarkovMobilityModel")
		.SetParent<MobilityModel> ()
		.SetGroupName ("Mobility")
		.AddConstructor<GaussMarkovMobilityModel> ()
		.AddAttribute ("Bounds","Bounds of the area to cruise.",BoxValue (Box (-100.0, 100.0, -100.0, 100.0, 0.0, 100.0)),MakeBoxAccessor (&GaussMarkovMobilityModel::m_bounds),MakeBoxChecker ())
		.AddAttribute ("TimeStep","Change current direction and speed after moving for this time.",TimeValue (Seconds (1.0)),MakeTimeAccessor (&GaussMarkovMobilityModel::m_timeStep),MakeTimeChecker ())
		.AddAttribute ("Alpha","A constant representing the tunable parameter in the Gauss-Markov model.",DoubleValue (1.0),MakeDoubleAccessor (&GaussMarkovMobilityModel::m_alpha),MakeDoubleChecker<double> ())
		.AddAttribute ("MeanVelocity","A random variable used to assign the average velocity.",StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=1.0]"),MakePointerAccessor (&GaussMarkovMobilityModel::m_rndMeanVelocity),MakePointerChecker<RandomVariableStream> ())
		.AddAttribute ("MeanDirection","A random variable used to assign the average direction.",StringValue ("ns3::UniformRandomVariable[Min=0.0|Max=6.283185307]"),MakePointerAccessor (&GaussMarkovMobilityModel::m_rndMeanDirection),MakePointerChecker<RandomVariableStream> ())
		.AddAttribute ("MeanPitch","A random variable used to assign the average pitch.",StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"),MakePointerAccessor (&GaussMarkovMobilityModel::m_rndMeanPitch),MakePointerChecker<RandomVariableStream> ())
		.AddAttribute ("NormalVelocity","A gaussian random variable used to calculate the next velocity value.",StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0|Bound=10.0]"),MakePointerAccessor (&GaussMarkovMobilityModel::m_normalVelocity),MakePointerChecker<NormalRandomVariable> ())
		.AddAttribute ("NormalDirection","A gaussian random variable used to calculate the next direction value.",StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0|Bound=10.0]"),MakePointerAccessor (&GaussMarkovMobilityModel::m_normalDirection),MakePointerChecker<NormalRandomVariable> ())
		.AddAttribute ("NormalPitch","A gaussian random variable used to calculate the next pitch value.",StringValue ("ns3::NormalRandomVariable[Mean=0.0|Variance=1.0|Bound=10.0]"),MakePointerAccessor (&GaussMarkovMobilityModel::m_normalPitch),MakePointerChecker<NormalRandomVariable> ())
		;

    return tid;
  }

  GaussMarkovMobilityModel::GaussMarkovMobilityModel ()
  {
    m_meanVelocity = 0.0;
    m_meanDirection = 0.0;
    m_meanPitch = 0.0;
    m_event = Simulator::ScheduleNow (&GaussMarkovMobilityModel::Start, this);
    m_helper.Unpause ();
    m_closestObstacle = -1;
  }

  void
  GaussMarkovMobilityModel::AddObstacle (const Box &obstacle)
  {
//    double fat = 50;//m_Velocity*m_timeStep.GetSeconds();
//    m_obstacles.push_back(Box(obstacle.xMin-fat,obstacle.xMax+fat,obstacle.yMin-fat,obstacle.yMax+fat,obstacle.zMin-fat,obstacle.zMax+fat));
    m_obstacles.push_back(obstacle);
  }

  void
  GaussMarkovMobilityModel::Start (void)
  {
    if (m_meanVelocity == 0.0)
      {
	//Initialize the mean velocity, direction, and pitch variables
	m_meanVelocity = m_rndMeanVelocity->GetValue ();
	m_meanDirection = m_rndMeanDirection->GetValue ();
	m_meanPitch = m_rndMeanPitch->GetValue ();
	double cosD = std::cos (m_meanDirection);
	double cosP = std::cos (m_meanPitch);
	double sinD = std::sin (m_meanDirection);
	double sinP = std::sin (m_meanPitch);
	//Initialize the starting velocity, direction, and pitch to be identical to the mean ones
	m_Velocity = m_meanVelocity;
	m_Direction = m_meanDirection;
	m_Pitch = m_meanPitch;
	//Set the velocity vector to give to the constant velocity helper
	m_helper.SetVelocity (Vector (m_Velocity*cosD*cosP, m_Velocity*sinD*cosP, m_Velocity*sinP));
      }
    m_helper.Update ();

    //Get the next values from the gaussian distributions for velocity, direction, and pitch
    double rv = m_normalVelocity->GetValue ();
    double rd = m_normalDirection->GetValue ();
    double rp = m_normalPitch->GetValue ();

    //Calculate the NEW velocity, direction, and pitch values using the Gauss-Markov formula:
    //newVal = alpha*oldVal + (1-alpha)*meanVal + sqrt(1-alpha^2)*rv
    //where rv is a random number from a normal (gaussian) distribution
    double one_minus_alpha = 1 - m_alpha;
    double sqrt_alpha = std::sqrt (1 - m_alpha*m_alpha);
    m_Velocity  = m_alpha * m_Velocity  + one_minus_alpha * m_meanVelocity  + sqrt_alpha * rv;
    m_Direction = m_alpha * m_Direction + one_minus_alpha * m_meanDirection + sqrt_alpha * rd;
    m_Pitch     = m_alpha * m_Pitch     + one_minus_alpha * m_meanPitch     + sqrt_alpha * rp;

    //Calculate the linear velocity vector to give to the constant velocity helper
    double cosDir = std::cos (m_Direction);
    double cosPit = std::cos (m_Pitch);
    double sinDir = std::sin (m_Direction);
    double sinPit = std::sin (m_Pitch);
    double vx = m_Velocity * cosDir * cosPit;
    double vy = m_Velocity * sinDir * cosPit;
    double vz = m_Velocity * sinPit;
    m_helper.SetVelocity (Vector (vx, vy, vz));

    m_helper.Unpause ();

    DoWalk (m_timeStep);
  }

  void
  GaussMarkovMobilityModel::DoWalk (Time delayLeft)
  {
    m_helper.UpdateWithBounds (m_bounds);
    Vector position = m_helper.GetCurrentPosition ();
    Vector speed = m_helper.GetVelocity ();

    double speedM = std::sqrt(speed.x*speed.x + speed.y*speed.y + speed.z*speed.z);

    Vector nextPosition = position;
    nextPosition.x += speed.x * delayLeft.GetSeconds ();
    nextPosition.y += speed.y * delayLeft.GetSeconds ();
    nextPosition.z += speed.z * delayLeft.GetSeconds ();
    if (delayLeft.GetSeconds () < 0.0) delayLeft = Seconds (1.0);

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
        m_event = Simulator::Schedule (delay_tmp, &GaussMarkovMobilityModel::Start, this);
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

        m_event = Simulator::Schedule (delay_tmp, &GaussMarkovMobilityModel::Rebound, this,
  				     delayLeft - delay_tmp);
      }
    NotifyCourseChange ();
  }

  void
  GaussMarkovMobilityModel::Rebound (Time delayLeft)
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
	m_meanDirection = M_PI - m_meanDirection;
	break;
      case Box::TOP:
      case Box::BOTTOM:
	speed.y = -speed.y;
	m_meanDirection = -m_meanDirection;
	break;
      case Box::UP:
      case Box::DOWN:
	speed.z = -speed.z;
	m_meanPitch = -m_meanPitch;
	break;
    }
    m_helper.SetVelocity (speed);
    m_helper.Unpause ();
    DoWalk (delayLeft);
  }

  void
  GaussMarkovMobilityModel::DoDispose (void)
  {
    // chain up
    MobilityModel::DoDispose ();
  }

  Vector
  GaussMarkovMobilityModel::DoGetPosition (void) const
  {
    m_helper.Update ();
    return m_helper.GetCurrentPosition ();
  }
  void
  GaussMarkovMobilityModel::DoSetPosition (const Vector &position)
  {
    m_helper.SetPosition (position);
    Simulator::Remove (m_event);
    m_event = Simulator::ScheduleNow (&GaussMarkovMobilityModel::Start, this);
  }
  Vector
  GaussMarkovMobilityModel::DoGetVelocity (void) const
  {
    return m_helper.GetVelocity ();
  }

  int64_t
  GaussMarkovMobilityModel::DoAssignStreams (int64_t stream)
  {
    m_rndMeanVelocity->SetStream (stream);
    m_normalVelocity->SetStream (stream + 1);
    m_rndMeanDirection->SetStream (stream + 2);
    m_normalDirection->SetStream (stream + 3);
    m_rndMeanPitch->SetStream (stream + 4);
    m_normalPitch->SetStream (stream + 5);
    return 6;
  }

} // namespace ns3

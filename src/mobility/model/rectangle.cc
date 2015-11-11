/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#include "rectangle.h"
#include "ns3/vector.h"
#include "ns3/assert.h"
#include "ns3/fatal-error.h"
#include <cmath>
#include <algorithm>
#include <sstream> 
#include <typeinfo>

namespace ns3 {

Rectangle::Rectangle (double _xMin, double _xMax,
                      double _yMin, double _yMax)
  : xMin (_xMin),
    xMax (_xMax),
    yMin (_yMin),
    yMax (_yMax)
{
}

Rectangle::Rectangle ()
  : xMin (0.0),
    xMax (0.0),
    yMin (0.0),
    yMax (0.0)
{
}

bool
Rectangle::IsInside (const Vector &position) const
{
  return
    position.x <= this->xMax && position.x >= this->xMin &&
    position.y <= this->yMax && position.y >= this->yMin;
}

bool
Rectangle::IsOutside (const Vector &position) const
{
  return !IsInside(position);
}

Rectangle::Side
Rectangle::GetClosestSide (const Vector &position) const
{
  double xMinDist = std::abs (position.x - this->xMin);
  double xMaxDist = std::abs (this->xMax - position.x);
  double yMinDist = std::abs (position.y - this->yMin);
  double yMaxDist = std::abs (this->yMax - position.y);
  double minX = std::min (xMinDist, xMaxDist);
  double minY = std::min (yMinDist, yMaxDist);
  if (minX < minY)
    {
      if (xMinDist < xMaxDist)
        {
          return LEFT;
        }
      else
        {
          return RIGHT;
        }
    }
  else
    {
      if (yMinDist < yMaxDist)
        {
          return BOTTOM;
        }
      else
        {
          return TOP;
        }
    }
}

Vector
Rectangle::CalculateIntersection (const Vector &current, const Vector &speed) const
{
  NS_ASSERT (IsInside (current));
  double xMaxY = current.y + (this->xMax - current.x) / speed.x * speed.y;
  double xMinY = current.y + (this->xMin - current.x) / speed.x * speed.y;
  double yMaxX = current.x + (this->yMax - current.y) / speed.y * speed.x;
  double yMinX = current.x + (this->yMin - current.y) / speed.y * speed.x;
  bool xMaxYOk = (xMaxY <= this->yMax && xMaxY >= this->yMin);
  bool xMinYOk = (xMinY <= this->yMax && xMinY >= this->yMin);
  bool yMaxXOk = (yMaxX <= this->xMax && yMaxX >= this->xMin);
  bool yMinXOk = (yMinX <= this->xMax && yMinX >= this->xMin);
  if (xMaxYOk && speed.x >= 0)
    {
      return Vector (this->xMax, xMaxY, 0.0);
    }
  else if (xMinYOk && speed.x <= 0)
    {
      return Vector (this->xMin, xMinY, 0.0);
    }
  else if (yMaxXOk && speed.y >= 0)
    {
      return Vector (yMaxX, this->yMax, 0.0);
    }
  else if (yMinXOk && speed.y <= 0)
    {
      return Vector (yMinX, this->yMin, 0.0);
    }
  else
    {
      NS_ASSERT (false);
      // quiet compiler
      return Vector (0.0, 0.0, 0.0);
    }
}

bool
Rectangle::DoesCollide (const Vector &current, const Vector &next, const Vector &speed) const
{
	NS_ASSERT(IsOutside (current));
	//Check if line intersects obstacle at all, disregarding direction of the line (node's trajectory)
	double x1 = current.x;
	double y1 = current.y;
	double x2 = next.x;
	double y2 = next.y;

	double m = ((y1 - y2) / (x1 - x2));
	double b = y1 - (m*x1);

	double c1 = (this->yMax - b)/m;
	double c2 = (this->yMin - b)/m;
	double c3 = (m*this->xMax)+b;
	double c4 = (m*this->xMin)+b;

	bool collides = false;
	if (this->xMin <= c1 && c1 <= this->xMax)
		collides = true;
	else if (this->xMin <= c2 && c2 <= this->xMax)
		collides = true;
	else if (this->yMin <= c3 && c3 <= this->yMax)
		collides = true;
	else if (this->yMin <= c4 && c4 <= this->yMax)
		collides = true;

	if (collides == false)
		return collides;

	//Check if node is approaching or distancing from obstacle
	double dt = .0000001;
	Vector pos1 = Vector (current.x + (speed.x*dt), current.y + (speed.y*dt), 0.0);

	int distanceScore = 0;
	if (fabs(pos1.x - this->xMin) < fabs(current.x - this->xMin))
		distanceScore++;
	if (fabs(pos1.x - this->xMax) < fabs(current.x - this->xMax))
		distanceScore++;
	if (fabs(pos1.y - this->yMax) < fabs(current.y - this->yMax))
		distanceScore++;
	if (fabs(pos1.y - this->yMin) < fabs(current.y - this->yMin))
		distanceScore++;
	if (distanceScore >= 2)
		return true;

	return false;
}

Vector
Rectangle::CollisionPoint (const Vector &current, const Vector &next, const Vector &speed) const
{
	NS_ASSERT (DoesCollide (current,next,speed));
	//find the line equation
	double x1 = current.x;
	double x2 = next.x;
	double y1 = current.y;
	double y2 = next.y;
	double m = ((y1 - y2) / (x1 - x2));
	double b = y1 - (m*x1);
	// Find all intersection points
	double c1 = (this->yMax - b)/m;
	double c2 = (this->yMin - b)/m;
	double c3 = (m*this->xMax)+b;
	double c4 = (m*this->xMin)+b;
	//Check which points are valid (contained in obstacles limits, max 2 points: i1 and i2)
	Vector i1 = Vector(0.0,0.0,0.0);
	Vector i2 = Vector(0.0,0.0,0.0);
	bool oneSet = false;
	if (this->xMin <= c1 && c1 <= this->xMax)
	{
		i1 = Vector (c1, this->yMax, 0.0);
		oneSet = true;
	}
	if (this->xMin <= c2 && c2 <= this->xMax)
	{
		if (oneSet == false) {
			i1 = Vector (c2, this->yMin, 0.0);
			oneSet = true;
		}else {
			i2 = Vector (c2, this->yMin, 0.0);
			oneSet = true;
		}
	}
	if (this->yMin <= c3 && c3 <= this->yMax)
	{
		if (oneSet == false)
		{
			i1 = Vector (this->xMax, c3, 0.0);
			oneSet = true;
		}else{
			i2 = Vector (this->xMax, c3, 0.0);
			oneSet = true;
		}
	}
	if (this->yMin <= c4 && c4 <= this->yMax)
	{
		if (oneSet == false) {
			i1 = Vector (this->xMin, c4, 0.0);
			oneSet = true;
		}else {
			i2 = Vector (this->xMin, c4, 0.0);
		}
	}
	//Check which intersection point is closer to the current location
	if (CalculateDistance(current, i1) <= CalculateDistance(current, i2))
	{
		return i1;
	}else{
		return i2;
	}

}

ATTRIBUTE_HELPER_CPP (Rectangle);

/**
 * \brief Stream insertion operator.
 *
 * \param os the stream
 * \param rectangle the rectangle
 * \returns a reference to the stream
 */
std::ostream &
operator << (std::ostream &os, const Rectangle &rectangle)
{
  os << rectangle.xMin << "|" << rectangle.xMax << "|" << rectangle.yMin << "|" << rectangle.yMax;
  return os;
}
/**
 * \brief Stream extraction operator.
 *
 * \param is the stream
 * \param rectangle the rectangle
 * \returns a reference to the stream
 */
std::istream &
operator >> (std::istream &is, Rectangle &rectangle)
{
  char c1, c2, c3;
  is >> rectangle.xMin >> c1 >> rectangle.xMax >> c2 >> rectangle.yMin >> c3 >> rectangle.yMax;
  if (c1 != '|' ||
      c2 != '|' ||
      c3 != '|')
    {
      is.setstate (std::ios_base::failbit);
    }
  return is;
}


} // namespace ns3

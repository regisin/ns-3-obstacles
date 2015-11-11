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
 *
 * Modifications made by: Paulo Regis <pregis@nevada.unr.edu>
 */
#ifndef BOX_H
#define BOX_H

#include "ns3/attribute.h"
#include "ns3/attribute-helper.h"
#include "ns3/vector.h"
#include "ns3/rectangle.h"

namespace ns3 {

/**
 * \ingroup mobility
 * \brief a 3d box
 * \see attribute_Box
 */
class Box
{
public:
  /**
   * Enum class to specify sides of a box
   */
  enum Side {
    RIGHT,
    LEFT,
    TOP,
    BOTTOM,
    UP,
    DOWN
  };
  /**
   * \param _xMin x coordinates of left boundary.
   * \param _xMax x coordinates of right boundary.
   * \param _yMin y coordinates of bottom boundary.
   * \param _yMax y coordinates of top boundary.
   * \param _zMin z coordinates of down boundary.
   * \param _zMax z coordinates of up boundary.
   *
   * Create a box.
   */
  Box (double _xMin, double _xMax,
       double _yMin, double _yMax,
       double _zMin, double _zMax);
  /**
   * Create a zero-sized box located at coordinates (0.0,0.0,0.0)
   */
  Box ();
  /**
   * \param position the position to test.
   * \returns true if the input position is located within the box, 
   *          false otherwise.
   *
   * This method compares the x, y, and z coordinates of the input position.
   */
  bool IsInside (const Vector &position) const;
  /**
   * \param position the position to test.
   * \return true if the input position is located outside the box,
   * 		 false otherwise.
   *
   * This method compares the x, y, and z coordinates of the input position.
   */
  bool IsOutside (const Vector &position) const;
  /**
   * \param position the position to test.
   * \returns the side of the cube the input position is closest to.
   *
   * This method compares the x, y, and z coordinates of the input position.
   */
  Side GetClosestSide (const Vector &position) const;
  /**
   * \param current the current position
   * \param speed the current speed
   * \return the intersection point between the rectangle and the current+speed vector.
   *
   * This method assumes that the current position is located _inside_
   * the rectangle and checks for this with an assert.
   * This method compares the x, y, and z coordinates of the input position.
   */
  Vector CalculateIntersection (const Vector &current, const Vector &speed) const;




  /**
   * \param current the current position
   * \param next the next position
   * \param speed the current speed
   * \return true if current+speed vector intersects with with the box
   *
   * This method assumes that the current position is located _outside_
   * the obstacle box and checks for this with an assert.
   */
  bool WillCollide (const Vector &current, const Vector &speed, Vector &collisionPoint) const;
  /**
   * \param current the current position
   * \param next the next position
   * \param speed the current speed
   * \return the distance traveled before collision between
   * the box and the current+speed vector occurs.
   *
   * This method assumes the vector current+speed collides with
   * the obstacle box and checks for this with an assert.
   * This method compares the x, y, and z coordinates of the input position.
   */
//  double CollisionDistance(const Vector &current, const Vector &next, const Vector &speed) const;











//  int GetIntersection(double fDst1, double fDst2, const Vector &current, const Vector &next, Vector &Hit) const;
//  int InBox(Vector Hit, Vector B1, Vector B2, const int Axis) const;



  /** The x coordinate of the left bound of the box */
  double xMin;
  /** The x coordinate of the right bound of the box */
  double xMax;
  /** The y coordinate of the bottom bound of the box */
  double yMin;
  /** The y coordinate of the top bound of the box */
  double yMax;
  /** The z coordinate of the down bound of the box */
  double zMin;
  /** The z coordinate of the up bound of the box */
  double zMax;
};

std::ostream &operator << (std::ostream &os, const Box &box);
std::istream &operator >> (std::istream &is, Box &box);

ATTRIBUTE_HELPER_HEADER (Box);

} // namespace ns3

#endif /* BOX_H */

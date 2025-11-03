// $CVSHeader: SnSim/Point2d.h,v 1.3.2.1 2005/05/11 13:55:17 cianci Exp $

/** \file       Point2d.h
 *  \brief      a point (x,y) in 2 dimensional euclidean space
 *  \author     $Author: cianci $
 *  \version    $Revision: 1.3.2.1 $
 *  \date       $Date: 2007-11-21 10:49:19 +0100 (Wed, 21 Nov 2007) $
 */

#ifndef _POINT2D_H
#define _POINT2D_H

#include <iostream>
#include <cmath>


//! \brief x,y coordinate in 2d euclidean space
class Point2d {
    
    public:

        double x;                       //!< x-coordinate
        double y;                       //!< y-coordinate

        //! \brief operator: formatting for output streams
        friend ostream &operator<<(ostream &os, const Point2d &p)
        { return os << "[" << p.x << "," << p.y << "]"; }

        //! \brief default constructor (initialized to the origin)
        Point2d( double xin=0, double yin=0 ) 
        {
            x = xin;
            y = yin;
        };

        //! \brief copy constructor
        Point2d( const Point2d &p ) 
        {
            x = p.x;
            y = p.y;
        };

        //! \brief destructor
        ~Point2d() {};

        //! \brief operator: assignment
        Point2d operator= (const Point2d p)
        { 
            x = p.x; 
            y = p.y;
            return *this; 
        }

        //! \brief distance to another Point2d
        double Distance(Point2d p)
        {
            return sqrt( (x-p.x)*(x-p.x) + (y-p.y)*(y-p.y) );
        }

        //! \brief angle to another Point2d
        double Angle(Point2d p)
        {
            return atan2( p.y-y, p.x-x );
        }
};


#endif // Point2d.h

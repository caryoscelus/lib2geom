/* operator functions for Geom::Point. */
#ifndef SEEN_Geom_POINT_OPS_H
#define SEEN_Geom_POINT_OPS_H

#include "point.h"

namespace Geom {

inline Point operator+(Point const &a, Point const &b)
{
    Point ret;
    for (int i = 0; i < 2; i++) {
        ret[i] = a[i] + b[i];
    }
    return ret;
}

inline Point operator-(Point const &a, Point const &b)
{
    Point ret;
    for (int i = 0; i < 2; i++) {
        ret[i] = a[i] - b[i];
    }
    return ret;
}

/** This is a rotation (sort of). */
inline Point operator^(Point const &a, Point const &b)
{
    Point const ret(a[0] * b[0] - a[1] * b[1],
                    a[1] * b[0] + a[0] * b[1]);
    return ret;
}

inline Point operator-(Point const &a)
{
    Point ret;
    for(unsigned i = 0; i < 2; i++) {
        ret[i] = -a[i];
    }
    return ret;
}

inline Point operator*(double const s, Point const &b)
{
    Point ret;
    for(int i = 0; i < 2; i++) {
        ret[i] = s * b[i];
    }
    return ret;
}

inline Point operator*(Point const &b, double const s)
{
    Point ret;
    for(int i = 0; i < 2; i++) {
        ret[i] = s * b[i];
    }
    return ret;
}

inline Point operator/(Point const &b, double const d)
{
    Point ret;
    for(int i = 0; i < 2; i++) {
        ret[i] = b[i] / d;
    }
    return ret;
}

inline Point operator/(double const d, Point const &b)
{
    Point ret;
    for(int i = 0; i < 2; i++) {
        ret[i] = d / b[i];
    }
    return ret;
}

inline bool operator==(Point const &a, Point const &b)
{
    return ( ( a[X] == b[X] ) && ( a[Y] == b[Y] ) );
}

inline bool operator!=(Point const &a, Point const &b)
{
    return ( ( a[X] != b[X] ) || ( a[Y] != b[Y] ) );
}

/** This is a lexicographical ordering for points.  It is remarkably useful for sweepline
 * algorithms*/
inline bool operator<=(Point const &a, Point const &b)
{
    return ( ( a[Y] < b[Y] ) ||
             (( a[Y] == b[Y] ) && ( a[X] < b[X] )));
}


} /* namespace Geom */


#endif /* !SEEN_Geom_POINT_OPS_H */

/*
  Local Variables:
  mode:c++
  c-file-style:"stroustrup"
  c-file-offsets:((innamespace . 0)(inline-open . 0)(case-label . +))
  indent-tabs-mode:nil
  fill-column:99
  End:
*/
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=8:softtabstop=4:encoding=utf-8:textwidth=99 :
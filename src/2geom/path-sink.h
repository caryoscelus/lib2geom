/**
 * \file
 * \brief  callback interface for SVG path data
 *//*
 * Copyright 2007 MenTaLguY <mental@rydia.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it either under the terms of the GNU Lesser General Public
 * License version 2.1 as published by the Free Software Foundation
 * (the "LGPL") or, at your option, under the terms of the Mozilla
 * Public License Version 1.1 (the "MPL"). If you do not alter this
 * notice, a recipient may use your version of this file under either
 * the MPL or the LGPL.
 *
 * You should have received a copy of the LGPL along with this library
 * in the file COPYING-LGPL-2.1; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 * You should have received a copy of the MPL along with this library
 * in the file COPYING-MPL-1.1
 *
 * The contents of this file are subject to the Mozilla Public License
 * Version 1.1 (the "License"); you may not use this file except in
 * compliance with the License. You may obtain a copy of the License at
 * http://www.mozilla.org/MPL/
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY
 * OF ANY KIND, either express or implied. See the LGPL or the MPL for
 * the specific language governing rights and limitations.
 *
 */

#ifndef LIB2GEOM_SEEN_PATH_SINK_H
#define LIB2GEOM_SEEN_PATH_SINK_H

#include <2geom/pathvector.h>
#include <2geom/curves.h>
#include <iterator>

namespace Geom {


/** @brief Callback interface for processing path data.
 *
 * PathSink provides an interface that allows one to easily write
 * code which processes path data, for instance when converting
 * between path formats used by different graphics libraries.
 *
 * To store a path in a new format, implement the virtual methods
 * for segments in a derived class and call feed().
 */
class PathSink {
public:
    /** Move to a different point without creating a segment.
     * Usually starts a new subpath. */
    virtual void moveTo(Point const &p) = 0;
    /// Output a line segment.
    virtual void lineTo(Point const &p) = 0;
    /// Output a quadratic Bezier segment.
    virtual void curveTo(Point const &c0, Point const &c1, Point const &p) = 0;
    /// Output a cubic Bezier segment.
    virtual void quadTo(Point const &c, Point const &p) = 0;
    /** Output an elliptical arc segment.
     * See the EllipticalArc class for the documentation of parameters. */
    virtual void arcTo(double rx, double ry, double angle,
                       bool large_arc, bool sweep, Point const &p) = 0;

    /// Close the current path with a line segment.
    virtual void closePath() = 0;
    /** Flush any internal state of the generator.
     * 
     * This call should implicitly finish the current subpath.
     * Calling this method should be idempotent, because the default
     * implementations of path() and pathvector() will call it
     * multiple times in a row. */
    virtual void flush() = 0;
    // Get the current point, e.g. where the initial point of the next segment will be.
    //virtual Point currentPoint() const = 0;

    /** Undo the last segment.
     * This method is optional.
     * @return true true if a segment was erased, false otherwise. */
    virtual bool backspace() { return false; }

    // these have a default implementation
    virtual void feed(Curve const &c, bool moveto_initial = true);
    /** Output a subpath.
     * Calls the appropriate segment methods according to the contents
     * of the passed subpath. You can override this function. */
    virtual void feed(Path const &p);
    /** Output a path.
     * Calls the appropriate segment methods according to the contents
     * of the passed path. You can override this function. */
    virtual void feed(PathVector const &v);
    /// Output an axis-aligned rectangle, using moveTo, lineTo and closePath.
    virtual void feed(Rect const &);

    virtual ~PathSink() {}
};

template <typename OutputIterator>
class PathIteratorSink : public PathSink {
public:
    explicit PathIteratorSink(OutputIterator out)
    : _in_path(false), _out(out) {}

    void moveTo(Point const &p) {
        flush();
        _path.start(p);
        _start_p = p;
        _in_path = true;
    }
//TODO: what if _in_path = false?

    void lineTo(Point const &p) {
        // check for implicit moveto, like in: "M 1,1 L 2,2 z l 2,2 z"
        if (!_in_path) {
            moveTo(_start_p);
        }
        _path.template appendNew<LineSegment>(p);
    }

    void quadTo(Point const &c, Point const &p) {
        // check for implicit moveto, like in: "M 1,1 L 2,2 z l 2,2 z"
        if (!_in_path) {
            moveTo(_start_p);
        }
        _path.template appendNew<QuadraticBezier>(c, p);
    }

    void curveTo(Point const &c0, Point const &c1, Point const &p) {
        // check for implicit moveto, like in: "M 1,1 L 2,2 z l 2,2 z"
        if (!_in_path) {
            moveTo(_start_p);
        }
        _path.template appendNew<CubicBezier>(c0, c1, p);
    }

    void arcTo(double rx, double ry, double angle,
               bool large_arc, bool sweep, Point const &p)
    {
        // check for implicit moveto, like in: "M 1,1 L 2,2 z l 2,2 z"
        if (!_in_path) {
            moveTo(_start_p);
        }
        _path.template appendNew<SVGEllipticalArc>(rx, ry, angle,
                                                 large_arc, sweep, p);
    }

    bool backspace()
    {
        if (_in_path && _path.size() > 0) {
            _path.erase_last();
            return true;
        }
        return false;
    }

    void append(Path const &other, Path::Stitching stitching = Path::NO_STITCHING)
    {
        if (!_in_path) {
            moveTo(other.initialPoint());
        }
        _path.append(other, stitching);
    }

    void closePath() {
        _path.close();
        flush();
    }

    void flush() {
        if (_in_path) {
            _in_path = false;
            *_out++ = _path;
            _path.clear();
        }
    }

    void path(Path const &other)
    {
        flush();
        *_out++ = other;
    }

protected:
    bool _in_path;
    OutputIterator _out;
    Path _path;
    Point _start_p;
};

typedef std::back_insert_iterator<PathVector> SubpathInserter;

class PathBuilder : public PathIteratorSink<SubpathInserter> {
private:
    PathVector _pathset;
public:
    PathBuilder() : PathIteratorSink<SubpathInserter>(SubpathInserter(_pathset)) {}
    PathVector const &peek() const {return _pathset;}
};

}

#endif
/*
  Local Variables:
  mode:c++
  c-file-style:"stroustrup"
  c-file-offsets:((innamespace . 0)(inline-open . 0)(case-label . +))
  indent-tabs-mode:nil
  fill-column:99
  End:
*/
// vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=8:softtabstop=4:fileencoding=utf-8:textwidth=99 :

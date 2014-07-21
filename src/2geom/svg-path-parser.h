/**
 * \file
 * \brief  parse SVG path specifications
 *
 * Copyright 2007 MenTaLguY <mental@rydia.net>
 * Copyright 2007 Aaron Spike <aaron@ekips.org>
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

#ifndef SEEN_SVG_PATH_PARSER_H
#define SEEN_SVG_PATH_PARSER_H

#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>
#include <2geom/exception.h>
#include <2geom/point.h>
#include <2geom/path-sink.h>

namespace Geom {

/** @brief SVG format path data parsing
 *
 * This class provides an interface to an SVG path data parser written in Ragel.
 * It supports parsing the path data either at once or block-by-block.
 * Use the parse() functions to parse complete data and the feed() and finish()
 * functions to parse partial data.
 */
class SVGPathParser {
public:
    SVGPathParser(PathSink &sink);

    /** @brief Reset internal state.
     * Discards the internal state associated with partially parsed data,
     * letting you start from scratch. Note that any partial data written
     * to the path sink is not affected - you need to clear it yourself. */
    void reset();

    /** @brief Parse a C-style string.
     * The path sink is flushed and the internal state is reset after this call.
     * Note that the state is not reset before this method, so you can use it to
     * process the last block of partial data.
     * @param str String to parse
     * @param len Length of string or -1 if null-terminated */
    void parse(char const *str, int len = -1);
    /** @brief Parse an STL string. */
    void parse(std::string const &s);

    /** @brief Parse a part of path data stored in a C-style string.
     * This method does not reset internal state, so it can be called multiple
     * times to parse successive blocks of a longer SVG path data string.
     * To finish parsing, call finish() after the final block or call parse()
     * with the last block of data.
     * @param str String to parse
     * @param len Length of string or -1 if null-terminated */
    void feed(char const *str, int len = -1);
    /** @brief Parse a part of path data stored in an STL string. */
    void feed(std::string const &s);

    /** @brief Finalize parsing.
     * After the last block of data was submitted with feed(), call this method
     * to finalize parsing, flush the path sink and reset internal state.
     * You should not call this after parse(). */
    void finish();

private:
    bool _absolute;
    Point _current;
    Point _initial;
    Point _cubic_tangent;
    Point _quad_tangent;
    std::vector<Coord> _params;
    PathSink &_sink;

    int cs;
    std::string _number_part;

    void _push(Coord value);
    Coord _pop();
    bool _pop_flag();
    Coord _pop_coord(Geom::Dim2 axis);
    Point _pop_point();
    void _moveTo(Point const &p);
    void _lineTo(Point const &p);
    void _curveTo(Point const &c0, Point const &c1, Point const &p);
    void _quadTo(Point const &c, Point const &p);
    void _arcTo(double rx, double ry, double angle,
                bool large_arc, bool sweep, Point const &p);
    void _closePath();

    void _parse(char const *str, char const *strend, bool finish);
};

void parse_svg_path(char const *str, PathSink &sink);
void parse_svg_path_file(FILE *fi, PathSink &sink);

inline void parse_svg_path(std::string const &str, PathSink &sink) {
    parse_svg_path(str.c_str(), sink);
}

inline PathVector parse_svg_path(char const *str) {
    PathVector ret;
    SubpathInserter iter(ret);
    PathIteratorSink<SubpathInserter> generator(iter);

    parse_svg_path(str, generator);
    return ret;
}

inline PathVector read_svgd_f(FILE * fi) {
    PathVector ret;
    SubpathInserter iter(ret);
    PathIteratorSink<SubpathInserter> generator(iter);

    parse_svg_path_file(fi, generator);
    return ret;
}

inline PathVector read_svgd(char const *filename) {
    FILE* fi = fopen(filename, "r");
    if(fi == NULL) throw(std::runtime_error("Error opening file"));
    PathVector out = read_svgd_f(fi);
    fclose(fi);
    return out;
}

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

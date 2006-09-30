#include "path-find-points-of-interest.h"
#include "cubic_bez_util.h"
#include "path-poly-fns.h"

using namespace Geom;

const double Geom_EPSILON = 1e-18; // taken from libnr.  Probably sqrt(MIN_FLOAT).

static std::vector<double>
cubicto_extrema (std::vector<double> x) {
    std::vector<double> result;
    
// see derivation in path-cubicto.

    const Geom::Coord a =   (x[0] - 3 * x[1] + 3 * x[2] - x[3]);
    const Geom::Coord b = 2*(x[1] - 2 * x[2] + x[3]);
    const Geom::Coord c =   (x[2] - x[3]);

    /*
     * s = (-b +/- sqrt (b * b - 4 * a * c)) / 2 * a;
     */
    if (fabs (a) < Geom_EPSILON) {
        /* s = -c / b */
        if (fabs (b) > Geom_EPSILON) {
            double s = -c / b;
            if ((s > 0.0) && (s < 1.0)) {
                result.push_back(1.0 - s);
            }
        }
    } else {
        /* s = (-b +/- sqrt (b * b - 4 * a * c)) / 2 * a; */
        const Geom::Coord D = b * b - 4 * a * c;
        if (D >= 0.0) {
            /* Have solution */
            double d = sqrt (D);
            double s = (-b + d) / (2 * a);
            if ((s > 0.0) && (s < 1.0)) {
                result.push_back(1.0 - s);
            }
            s = (-b - d) / (2 * a);
            if ((s > 0.0) && (s < 1.0)) {
                result.push_back(1.0 - s);
            }
        }
    }
    return result;
}

/*** find_vector_extreme_points
 * 
 */
std::vector<Geom::Path::Location> find_vector_extreme_points(Geom::Path const & p, Geom::Point dir) {
    std::vector<Geom::Path::Location> result;

    for(Geom::Path::const_iterator iter(p.begin()), end(p.end()); iter != end; ++iter) {
        switch(iter.cmd()) {
        case Geom::lineto:
            break;
        case Geom::quadto:
            break;
        case Geom::cubicto:
        {
            std::vector<double> x;
            for(int i = 0; i < 4; i++)
                x.push_back(dot((*iter)[i],dir));
            std::vector<double> ts = cubicto_extrema(x);
            for(std::vector<double>::iterator it = ts.begin();
                it != ts.end();
                ++it) {
                result.push_back(Geom::Path::Location(iter, *it));
            }
            break;
        }
        default:
            break;
        }
    }
    
    return result;
}

/*** find_cusp_points
 * 
 */
std::vector<Geom::Path::Location>
find_cusp_points(Geom::Path const & p, int dim) {
    std::vector<Geom::Path::Location> result;

    for(Geom::Path::const_iterator iter(p.begin()), end(p.end()); iter != end; ++iter) {
        switch(iter.cmd()) {
        case Geom::lineto:
            break;
        case Geom::cubicto:
        {
            std::vector<double> x;
            for(int i = 0; i < 4; i++)
                x.push_back((*iter)[i][dim]);
            std::vector<double> ts = cubicto_extrema(x);
            for(std::vector<double>::iterator it = ts.begin();
                it != ts.end();
                ++it) {
                result.push_back(Geom::Path::Location(iter, *it));
            }
            break;
        }
        default:
            break;
        }
    }
    
    return result;
}



/*** find_inflection_points
 * 
 */
std::vector<Geom::Path::Location>
find_inflection_points(Geom::Path const & p) {
    /*std::vector<Geom::Path::Location> result;

    for(Geom::Path::const_iterator iter(p.begin()), end(p.end()); iter != end; ++iter) {
        switch(iter.cmd()) {
        case Geom::lineto:
            break;
        case Geom::cubicto:
        {
            Geom::Point pc[4];
            for(int i = 0; i < 4; i++)
                pc[i] = Geom::Point(0,0);

            
            cubic_bezier_poly_coeff((*iter).begin(), pc);
            
            Geom::Point aT = -rot90(pc[3]);
            Geom::Point bT = -rot90(pc[2]);
            double t_cusp = -dot(aT, pc[1])/(2*dot(aT, pc[2]));
            if(t_cusp > 0 && t_cusp < 1)
              result.push_back(Geom::Path::Location(iter, t_cusp));
            double det = t_cusp*t_cusp - dot(bT, pc[1])/(3*dot(aT, pc[2]));
            if(det >= 0) {
                det = sqrt(det);
                double t1 = t_cusp - det;
                double t2 = t_cusp + det;
                //printf("%g %g %g %g\n", t_cusp, det, t1, t2);
                if(t1 > 0 && t1 < 1)
                    result.push_back(Geom::Path::Location(iter, t1));
                if(t2 > 0 && t2 < 1)
                    result.push_back(Geom::Path::Location(iter, t2));
            }
            break;
        }
        default:
            break;
        }
    }
    
    return result;*/
}

/*** find_flat_points
 * 
 */
std::vector<Geom::Path::Location>
find_flat_points(Geom::Path const & p) {
    std::vector<Geom::Path::Location> result;

    for(Geom::Path::const_iterator iter(p.begin()), end(p.end()); iter != end; ++iter) {
        switch(iter.cmd()) {
        case Geom::lineto:
            break;
        case Geom::cubicto:
        {
            Geom::Point pc[4];
            for(int i = 0; i < 4; i++)
                pc[i] = Geom::Point(0,0);
            
            Geom::Point rhat = unit_vector((*iter)[1] - (*iter)[0]);
            Geom::Point shat = rot90(rhat);
            double s3 = dot((*iter)[2] - (*iter)[0], shat);
            
            double t1 = 2*sqrt(1/(3*fabs(s3)));
            if(t1 > 0 && t1 < 1)
                result.push_back(Geom::Path::Location(iter, t1));
            break;
        }
        default:
            break;
        }
    }
    
    return result;
}

/*** find_maximal_curvature_points
 * 
 */
std::vector<Geom::Path::Location>
find_maximal_curvature_points(Geom::Path const & p) {
    std::vector<Geom::Path::Location> result;

    for(Geom::Path::const_iterator iter(p.begin()), end(p.end()); iter != end; ++iter) {
        switch(iter.cmd()) {
        case Geom::lineto:
            break;
        case Geom::quadto:
        case Geom::cubicto:
        {
            Poly Bx = get_parametric_poly(*iter, X); // poly version of bezier (0-1)
            Poly By = get_parametric_poly(*iter, Y);
            Poly dBx = derivative(Bx);
            Poly dBy = derivative(By);
            Poly ddBx = derivative(dBx);
            Poly ddBy = derivative(dBy);
            Poly dcurl = ddBy*dBx -ddBx*dBy;
            Poly t2 = dBx*dBx + dBy*dBy;
            Poly curvature_num = 2*derivative(dcurl)*t2 - dcurl;
            curvature_num.normalize();
            std::cout << curvature_num << std::endl;
            if(curvature_num.degree() > 0) {
                std::vector<double> possible = solve_reals(curvature_num);
                for(unsigned i = 0; i < possible.size(); i++) {
                    const double t = polish_root(curvature_num, possible[i], 1e-6);
                    
                    if((0 < t) && (t < 1)) {
                        printf("%g %g", t, curvature_num.eval(t));
                        result.push_back(Geom::Path::Location(iter, t));
                    }
                }
                printf("\n");
            }
            break;
        }
        default:
            break;
        }
    }
    return result;
}

/*
  Local Variables:
  mode:c++
  c-file-style:"stroustrup"
  c-file-offsets:((innamespace . 0)(substatement-open . 0))
  indent-tabs-mode:nil
  c-brace-offset:0
  fill-column:99
  End:
  vim: filetype=cpp:expandtab:shiftwidth=4:tabstop=8:softtabstop=4 :
*/

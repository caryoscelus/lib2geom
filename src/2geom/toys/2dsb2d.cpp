#include "d2.h"
#include "sbasis.h"
#include "sbasis-2d.h"
#include "bezier-to-sbasis.h"
#include "transforms.h"

#include "path-cairo.h"
#include "toy-framework.h"
#include "path.h"
#include "svg-path-parser.h"

#include <vector>
using std::vector;
using namespace Geom;

unsigned total_pieces_sub;
unsigned total_pieces_inc;

class Sb2d2: public Toy {
    Path path_a;
    D2<SBasis2d> sb2;
    Piecewise<D2<SBasis> >  path_a_pw;
    
    void draw(cairo_t *cr, std::ostringstream *notify, int width, int height, bool save) {
        Geom::Point dir(1,-2);
        for(unsigned dim = 0; dim < 2; dim++) {
            Geom::Point dir(0,0);
            dir[dim] = 1;
            for(unsigned vi = 0; vi < sb2[dim].vs; vi++)
                for(unsigned ui = 0; ui < sb2[dim].us; ui++)
                    for(unsigned iv = 0; iv < 2; iv++)
                        for(unsigned iu = 0; iu < 2; iu++) {
                            unsigned corner = iu + 2*iv;
                            unsigned i = ui + vi*sb2[dim].us;
                            Geom::Point base((2*(iu+ui)/(2.*ui+1)+1)*width/4.,
                                             (2*(iv+vi)/(2.*vi+1)+1)*width/4.);
                            if(vi == 0 && ui == 0) {
                                base = Geom::Point(width/4., width/4.);
                            }
                            double dl = dot((handles[corner+4*i] - base), dir)/dot(dir,dir);
                            sb2[dim][i][corner] = dl/(width/2)*pow(4.0,ui+vi);
                        }
        }
        cairo_2dsb2d(cr, sb2, dir*0.1, width);
        cairo_set_source_rgba (cr, 0., 0., 0, 0.5);
        cairo_stroke(cr);
        for(unsigned i = 0; i < path_a_pw.size(); i++) {
            D2<SBasis> B = path_a_pw[i];
            //const int depth = sb2[0].us*sb2[0].vs;
            //const int surface_handles = 4*depth;
            //D2<SBasis> B = handles_to_sbasis<3>(handles.begin() + surface_handles);
            cairo_md_sb(cr, B);
            for(unsigned dim = 0; dim < 2; dim++) {
                std::vector<double> r = roots(B[dim]);
                for(unsigned i = 0; i < r.size(); i++)
                    draw_cross(cr, B(r[i]));
                r = roots(Linear(width/4) - B[dim]);
                for(unsigned i = 0; i < r.size(); i++)
                    draw_cross(cr, B(r[i]));
            }
            cairo_set_source_rgba (cr, 0., 0.125, 0, 1);
            cairo_stroke(cr);
            B *= (4./width);
            D2<SBasis> tB = compose_each(sb2, B);
            B = B*(width/2) + Geom::Point(width/4, width/4);
            tB = tB*(width/2) + Geom::Point(width/4, width/4);
            
            cairo_md_sb(cr, tB);
        }
        
        //*notify << "bo = " << sb2.index(0,0);

        Toy::draw(cr, notify, width, height, save);
    }
    void first_time(int argc, char** argv) {
        const char *path_a_name="star.svgd";
        if(argc > 1)
            path_a_name = argv[1];
        std::vector<Path> paths_a = read_svgd(path_a_name);
        assert(paths_a.size() > 0);
        path_a = paths_a[0];
        Rect bounds = path_a[0].boundsFast();
        std::cout << bounds.min() <<std::endl;
        path_a = path_a * Matrix(Translate(-bounds.min()));
        double extreme = std::max(bounds.width(), bounds.height());
        path_a = path_a * Scale(40./extreme);
        
        path_a_pw = path_a.toPwSb();
        for(unsigned dim = 0; dim < 2; dim++) {
            sb2[dim].us = 2;
            sb2[dim].vs = 2;
            const int depth = sb2[dim].us*sb2[dim].vs;
            sb2[dim].resize(depth, Linear2d(0));
        }
        
        handles.resize(sb2[0].vs*sb2[0].us*4);
        
        
    }
    virtual void resize_canvas(Geom::Rect const & s) {
        double width = s[0].extent();
        unsigned ii = 0;
        for(unsigned vi = 0; vi < sb2[0].vs; vi++)
            for(unsigned ui = 0; ui < sb2[0].us; ui++)
                for(unsigned iv = 0; iv < 2; iv++)
                    for(unsigned iu = 0; iu < 2; iu++)
                        handles[ii++] = Geom::Point((2*(iu+ui)/(2.*ui+1)+1)*width/4.,
                                                    (2*(iv+vi)/(2.*vi+1)+1)*width/4.);
        
    }
};

int main(int argc, char **argv) {
    init(argc, argv, new Sb2d2);
    return 0;
}

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
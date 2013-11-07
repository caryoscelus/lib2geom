/*
 * Test performance of
 *
 * Copyright (C) Authors 2007-2013
 * Authors:
 *       Johan Engelen   <j.b.c.engelen@alumnus.utwente.nl>
 *       Steren Giannini <steren.giannini@gmail.com>
 *
 * the test part was taken from Inkscape's Bend Path LPE (and simplified a bit)
 *
 * Released under GNU GPL, read the file 'COPYING' for more information
 */


#include <iostream>
#include <ctime>

#include "2geom/svg-path-parser.h"
#include "2geom/pathvector.h"
#include "2geom/path.h"
#include "2geom/d2.h"
#include "2geom/piecewise.h"
#include "2geom/sbasis.h"
using namespace Geom;

static char const *path_str =
    "M -460,523.79076 C -460.65718,514.47985 -460.79228,505.75495 -460.43635,497.52584 -460.08041,489.29674 "
    "-459.23345,481.56342 -457.92649,474.23566 -456.61954,466.9079 -454.85259,459.9857 -452.65671,453.37885 "
    "-450.46083,446.77199 -447.836,440.48046 -444.81328,434.41405 -441.79056,428.34764 -438.36994,422.50633 "
    "-434.58247,416.79991 -430.79501,411.0935 -426.64069,405.52196 -422.15057,399.99508 -417.66045,394.4682 "
    "-412.83452,388.98598 -407.70384,383.4582 -402.57316,377.93041 -397.13772,372.35705 -391.42857,366.6479 "
    "-378.14065,353.35999 -362.09447,346.86737 -344.03764,345.52929 -325.98081,344.19121 -305.91333,348.00766 "
    "-284.58283,355.33787 -263.25234,362.66809 -240.65882,373.51207 -217.5499,386.22904 -194.44098,398.94602 "
    "-170.81666,413.53598 -147.42457,428.35818 -124.03248,443.18037 -100.87262,458.2348 -78.692608,471.88067 "
    "-56.512594,485.52655 -35.312431,497.76389 -15.839741,506.95191 3.632948,516.13994 21.378163,522.27865 "
    "36.648281,523.72729 51.918398,525.17592 64.713417,521.93448 74.285714,512.36218 76.124134,510.52376 "
    "78.521676,506.73937 81.393111,501.43833 84.264546,496.13729 87.609874,489.31958 91.343869,481.41453 "
    "95.077864,473.50947 99.200524,464.51706 103.62662,454.86661 108.05272,445.21615 112.78226,434.90765 "
    "117.73001,424.37041 122.67776,413.83317 127.84371,403.0672 133.14266,392.50178 138.4416,381.93637 "
    "143.87352,371.57153 149.3532,361.83656 154.83288,352.10159 160.36031,342.99649 165.85028,334.95058 "
    "171.34024,326.90466 176.79273,319.91793 182.12252,314.41968 189.27211,307.04412 196.34366,301.16926 "
    "203.28385,296.57404 210.22405,291.97882 217.03288,288.66325 223.65704,286.40628 230.2812,284.1493 "
    "236.72068,282.95093 242.92215,282.59009 249.12362,282.22926 255.08709,282.70597 260.75924,283.79918 "
    "266.43138,284.89239 271.8122,286.60209 276.84836,288.70724 281.88453,290.81239 286.57604,293.31298 "
    "290.86959,295.98796 295.16313,298.66294 299.0587,301.51232 302.50297,304.31504 305.94724,307.11776 "
    "308.94022,309.87382 311.42857,312.36218 317.70879,318.6424 325.20531,326.50894 333.6587,335.29576 "
    "342.11209,344.08258 351.52235,353.78967 361.63006,363.75101 371.73777,373.71235 382.54293,383.92792 "
    "393.78611,393.7317 405.02929,403.53547 416.71049,412.92744 428.57029,421.24156 440.43008,429.55569 "
    "452.46847,436.79197 464.42603,442.28436 476.38359,447.77676 488.26032,451.52527 499.7968,452.86385 "
    "511.33327,454.20243 522.52948,453.13109 533.12602,448.98378 543.72255,444.83647 553.7194,437.61319 "
    "562.85714,426.6479";

static char const *bend_str =
    "M -489.19983,473.24818 C -478.58599,460.43092 -465.31914,447.40133 -450.91614,434.95746 -436.51315,422.51359 "
    "-420.97403,410.65543 -405.81565,400.18102 -390.65727,389.70661 -375.87963,380.61595 -362.99962,373.70708 "
    "-350.11961,366.79821 -339.13723,362.07112 -331.56936,360.32386 -323.49864,358.4605 -311.32246,357.96153 "
    "-296.07049,358.53595 -280.81852,359.11038 -262.49077,360.7582 -242.11692,363.18844 -221.74308,365.61867 "
    "-199.32313,368.83132 -175.88677,372.53539 -152.45042,376.23947 -127.99765,380.43497 -103.55815,384.83091 "
    "-90.798706,387.12596 -78.042879,389.47564 -65.437198,391.83855 -52.831517,394.20146 -40.375984,396.57759 "
    "-28.217127,398.92554 -16.058269,401.27349 -4.1960885,403.59326 7.2228872,405.84343 18.641863,408.09361 "
    "29.617633,410.27419 40.00367,412.34378 50.09967,414.35557 59.638414,416.26247 68.485308,418.02645 "
    "77.332201,419.79043 85.487246,421.41149 92.815847,422.85159 100.14445,424.29169 106.64661,425.55083 "
    "112.18773,426.59098 117.72885,427.63113 122.30894,428.45228 125.7934,429.01641 134.20502,430.37823 "
    "142.47705,429.25391 150.64115,426.35391 158.80525,423.45391 166.86142,418.77822 174.84134,413.03729 "
    "182.82125,407.29637 190.72491,400.49022 198.58398,393.32929 206.44305,386.16836 214.25752,378.65265 "
    "222.05908,371.49262 227.06421,366.89906 232.06402,362.4519 237.06688,358.33874 242.06974,354.22557 "
    "247.07564,350.44641 252.09294,347.18884 257.11025,343.93127 262.13896,341.1953 267.18744,339.16853 "
    "272.23593,337.14176 277.30418,335.82419 282.40056,335.40342 293.00641,334.52778 302.22965,341.02775 "
    "310.80162,351.57579 319.37358,362.12384 327.29427,376.71996 335.29503,392.03663 343.29579,407.3533 "
    "351.37662,423.39051 360.26886,436.82075 369.1611,450.25099 378.86474,461.07425 390.11114,465.96301 "
    "401.98566,471.1248 423.45174,470.72496 449.88663,467.14319 476.32151,463.56143 507.72518,456.79775 "
    "539.47487,449.23188 571.22455,441.66601 603.32025,433.29795 631.13918,426.50743 658.9581,419.7169 "
    "688.21455,391.64677 702.85715,390.39104";


// adapted from Inkscape's BendPath LPE
Geom::Piecewise<Geom::D2<Geom::SBasis> > doEffect_pwd2(Geom::Piecewise<Geom::D2<Geom::SBasis> > const &pwd2_in,
                                                       Piecewise<D2<SBasis> > const &bend_path)
{
    Piecewise<D2<SBasis> > uskeleton = arc_length_parametrization(bend_path, 2, .1);
    uskeleton = remove_short_cuts(uskeleton, .01);
    Piecewise<D2<SBasis> > n = rot90(derivative(uskeleton));
    n = force_continuity(remove_short_cuts(n, .1));

    D2<Piecewise<SBasis> > patternd2 = make_cuts_independent(pwd2_in);
    Piecewise<SBasis> x = Piecewise<SBasis>(patternd2[1]);
    Piecewise<SBasis> y = Piecewise<SBasis>(patternd2[0]);

    Geom::OptRect bbox = bounds_exact(pwd2_in);
    Interval bboxHorizontal = (*bbox)[Geom::X];
    Interval bboxVertical = (*bbox)[Geom::Y];

    x -= bboxHorizontal.min();
    y -= bboxVertical.middle();

    double scaling = uskeleton.cuts.back() / bboxHorizontal.extent();

    if (scaling != 1.0) {
        x *= scaling;
        y *= scaling;
    }

    Piecewise<D2<SBasis> > output = compose(uskeleton, x) + y * compose(n, x);
    return output;
}


int main()
{
    for (int rep = 0; rep < 3; rep++) {
        const int num_repeats = 100;
        std::vector<Path> path1 = parse_svg_path(path_str);
        std::vector<Path> path2 = parse_svg_path(bend_str);
        Piecewise<D2<SBasis> > pwd2_path = path1[0].toPwSb();
        Piecewise<D2<SBasis> > pwd2_bend = path2[0].toPwSb();
        std::clock_t start = std::clock();
        for (int i = 0; i < num_repeats; i++) {
            Geom::Piecewise<Geom::D2<Geom::SBasis> > result = doEffect_pwd2(pwd2_path, pwd2_bend);
        }
        std::clock_t stop = std::clock();
        std::cout << "Bend paths (" << num_repeats << "x): " << (stop - start) * (1000. / CLOCKS_PER_SEC) << " ms "
                  << std::endl;
    }
}



//
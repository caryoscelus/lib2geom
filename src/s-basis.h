#ifndef SEEN_SBASIS_H
#define SEEN_SBASIS_H
#include <vector>
#include <map>
#include <cassert>
#include <algorithm>
#include <iostream>
#include <math.h>

namespace Geom{

class Hat{
public:
    Hat () {}
    Hat(double d) :d(d) {}
    operator double() const { return d; }
    double d;
};

class Tri{
public:
    Tri () {}
    Tri(double d) :d(d) {}
    operator double() const { return d; }
    double d;
};

class Linear{
public:
    double a[2];
    Linear() {}
    Linear(double aa, double b) {a[0] = aa; a[1] = b;}
    Linear(Hat h, Tri t) {
        a[0] = double(h) - double(t)/2; 
        a[1] = double(h) + double(t)/2;
    }

    Linear(Hat h) {
        a[0] = double(h); 
        a[1] = double(h);
    }

    double operator[](const int i) const {
        assert(i >= 0);
        assert(i < 2);
        return a[i];
    }
    double& operator[](const int i) {
        assert(i >= 0);
        assert(i < 2);
        return a[i];
    }
    double point_at(double t) {
        return (a[0]*(1-t) + a[1]*t);
    }
    double
    operator()(double t) {
        return (a[0]*(1-t) + a[1]*t);
    }
    operator Tri() const {
        return a[1] - a[0];
    }
    operator Hat() const {
        return (a[1] + a[0])/2;
    }
    double apply(double t) { return (1-t)*a[0] + t*a[1];}
    
    bool zero() const { return a[0] == 0 && a[1] == 0; }
    bool is_finite() const;
};

inline Linear operator-(Linear const &a) {
    return Linear(-a.a[0], -a.a[1]);
}
inline Linear operator+(Linear const & a, Linear const & b) {
    return Linear(a[0] + b[0], a[1] + b[1]);
}
inline Linear operator-(Linear const & a, Linear const & b) {
    return Linear(a[0] - b[0], a[1] - b[1]);
}
inline Linear& operator+=(Linear & a, Linear const & b) {
    a[0] += b[0];
    a[1] += b[1];
    return a;
}
inline Linear& operator-=(Linear & a, Linear const & b) {
    a[0] -= b[0];
    a[1] -= b[1];
    return a;
}
inline bool operator==(Linear const & a, Linear const & b) {
    return a[0] == b[0] &&
        a[1] == b[1];
}
inline bool operator!=(Linear const & a, Linear const & b) {
    return a[0] != b[0] ||
        a[1] != b[1];
}
inline Linear operator*(double const a, Linear const & b) {
    return Linear(a*b[0], a*b[1]);
}

inline Linear
reverse(Linear const &b) {
    return Linear(b[1], b[0]);
}

/*** An empty SBasis is identically 0. */
class SBasis : public std::vector<Linear>{
public:
    SBasis() {}
    SBasis(SBasis const & a) :
        std::vector<Linear>(a)
    {}
    SBasis(Linear const & bo) {
        push_back(bo);
    }
    
    double point_at(double t) const {
        double s = t*(1-t);
        double p0 = 0, p1 = 0;
        double sk = 1;
        int k = 0;
//TODO: rewrite as horner
        for(int k = 0; k < size(); k++) {
            p0 += sk*(*this)[k][0];
            p1 += sk*(*this)[k][1];
            sk *= s;
        }
        return (1-t)*p0 + t*p1;
    }
    double operator()(double t) const {
        return point_at(t);
    }
    //TODO: Why are these ops in the class while the others aren't? header even?
    SBasis operator+(const SBasis& p) const {
        SBasis result;
        const unsigned out_size = std::max(size(), p.size());
        const unsigned min_size = std::min(size(), p.size());
        //result.reserve(out_size);
        
        for(unsigned i = 0; i < min_size; i++) {
            result.push_back((*this)[i] + p[i]);
        }
        for(unsigned i = min_size; i < size(); i++)
            result.push_back((*this)[i]);
        for(unsigned i = min_size; i < p.size(); i++)
            result.push_back(p[i]);
        assert(result.size() == out_size);
        return result;
    }
    //TODO: Why are these ops in the class while the others aren't? header even?
    SBasis operator-(const SBasis& p) const {
        SBasis result;
        const unsigned out_size = std::max(size(), p.size());
        const unsigned min_size = std::min(size(), p.size());
        //result.reserve(out_size);
        
        for(unsigned i = 0; i < min_size; i++) {
            result.push_back((*this)[i] - p[i]);
        }
        for(unsigned i = min_size; i < size(); i++)
            result.push_back((*this)[i]);
        for(unsigned i = min_size; i < p.size(); i++)
            result.push_back(-p[i]);
        assert(result.size() == out_size);
        return result;
    }

    void clear() {
        fill(begin(), end(), Linear(0,0));
    }
    
    void normalize(); // remove extra zeros

    double tail_error(unsigned tail) const;
    
    void truncate(unsigned k);

// compute f(g)
    SBasis
    operator()(SBasis const & g) const;
    
    Linear&
    operator[](unsigned i) {
        //assert(i < size());
        return this->at(i);
        //return std::vector<Linear>::operator[](i);
    }
    
    Linear
    operator[](unsigned i) const {
        assert(i < size());
        return std::vector<Linear>::operator[](i);
    }
    bool is_finite() const;
};

inline SBasis operator-(const SBasis& p) {
    if(p.empty()) return SBasis();
    SBasis result;
    result.reserve(p.size());
        
    for(unsigned i = 0; i < p.size(); i++) {
        result.push_back(-p[i]);
    }
    return result;
}

inline SBasis operator-(Linear const & bo, const SBasis& p) {
    if(p.empty()) return bo;
    SBasis result;
    result.reserve(p.size());
        
    for(unsigned i = 0; i < p.size(); i++) {
        result.push_back(-p[i]);
    }
    result[0] += bo;
    return result;
   
}

inline SBasis& operator+=(SBasis& a, const SBasis& b) {
    const unsigned out_size = std::max(a.size(), b.size());
    const unsigned min_size = std::min(a.size(), b.size());
    a.reserve(out_size);
        
    for(unsigned i = 0; i < min_size; i++)
        a[i] += b[i];
    for(unsigned i = min_size; i < b.size(); i++)
        a.push_back(b[i]);
    
    assert(a.size() == out_size);
    return a;
}

inline SBasis& operator-=(SBasis& a, const SBasis& b) {
    const unsigned out_size = std::max(a.size(), b.size());
    const unsigned min_size = std::min(a.size(), b.size());
    a.reserve(out_size);
        
    for(unsigned i = 0; i < min_size; i++)
        a[i] -= b[i];
    for(unsigned i = min_size; i < b.size(); i++)
        a.push_back(-b[i]);
    
    assert(a.size() == out_size);
    return a;
}

inline SBasis& operator+=(SBasis& a, const Linear& b) {
    if(a.empty())
        a.push_back(b);
    else
        a[0] += b;
    return a;
}

inline SBasis& operator-=(SBasis& a, const Linear& b) {
    if(a.empty())
        a.push_back(-b);
    else
        a[0] -= b;
    return a;
}

inline SBasis& operator+=(SBasis& a, double b) {
    if(a.empty())
        a.push_back(Linear(b,b));
    else {
        a[0][0] += double(b);
        a[0][1] += double(b);
    }
    return a;
}

//TODO: Mutates the SBasis?!?!
inline SBasis operator+(Linear b, SBasis a) {
    if(a.empty())
        a.push_back(b);
    else {
        a[0] += b;
    }
    return a;
}

//TODO: Mutates the SBasis?!?!
inline SBasis operator+(double b, SBasis a) {
    if(a.empty())
        a.push_back(Linear(b,b));
    else {
        a[0][0] += double(b);
        a[0][1] += double(b);
    }
    return a;
}

inline SBasis& operator-=(SBasis& a, double b) {
    if(a.empty())
        a.push_back(Linear(-b, -b));
    else {
        a[0][0] -= double(b);
        a[0][1] -= double(b);
    }
    return a;
}

inline SBasis& operator*=(SBasis& a, double b) {
    for(unsigned i = 0; i < a.size(); i++) {
        a[i][0] *= b;
        a[i][1] *= b;
    }
    return a;
}

inline SBasis& operator/=(SBasis& a, double b) {
    for(unsigned i = 0; i < a.size(); i++) {
        a[i][0] /= b;
        a[i][1] /= b;
    }
    return a;
}

SBasis operator*(double k, SBasis const &a);
SBasis operator*(SBasis const &a, SBasis const &b);
//TODO: division equivalent?

SBasis shift(SBasis const &a, int sh);

SBasis shift(Linear const &a, int sh);

SBasis truncate(SBasis const &a, unsigned terms);

SBasis multiply(SBasis const &a, SBasis const &b);

SBasis integral(SBasis const &c);

SBasis derivative(SBasis const &a);

SBasis sqrt(SBasis const &a, int k);

// return a kth order approx to 1/a)
SBasis reciprocal(Linear const &a, int k);

SBasis divide(SBasis const &a, SBasis const &b, int k);

//TODO: remove above decleration of same function
inline SBasis
operator*(SBasis const & a, SBasis const & b) {
    return multiply(a, b);
}

inline SBasis& operator*=(SBasis& a, SBasis const & b) {
    a = multiply(a, b);
    return a;
}

//valuation: degree of the first non zero coefficient.
inline unsigned 
valuation(SBasis const &a, double tol=0){
    int val=0;
    while( val<a.size() && 
           fabs(a[val][0])<tol &&
           fabs(a[val][1])<tol )
        val++;
    return val;
}

// a(b(t))
SBasis compose(SBasis const &a, SBasis const &b);
SBasis compose(SBasis const &a, SBasis const &b, unsigned k);
SBasis inverse(SBasis a, int k);
//compose_inverse(f,g)=compose(f,inverse(g)), but is numerically more stable in some good cases...
SBasis compose_inverse(SBasis const &f, SBasis const &g, unsigned order, double tol=1e-7);

inline SBasis portion(const SBasis &t, double from, double to) { return compose(t, Linear(from, to)); }

// compute f(g)
inline SBasis
SBasis::operator()(SBasis const & g) const {
    return compose(*this, g);
}
 
inline std::ostream &operator<< (std::ostream &out_file, const Linear &bo) {
    out_file << "{" << bo[0] << ", " << bo[1] << "}";
    return out_file;
}

inline std::ostream &operator<< (std::ostream &out_file, const SBasis & p) {
    for(int i = 0; i < p.size(); i++) {
        out_file << p[i] << "s^" << i << " + ";
    }
    return out_file;
}

SBasis sin(Linear bo, int k);
SBasis cos(Linear bo, int k);

SBasis reverse(SBasis const &s);

//void bounds(SBasis const & s, double &lo, double &hi);
void bounds(SBasis const & s, double &lo, double &hi,int order=0);
void local_bounds(SBasis const & s, double t0, double t1, double &lo, double &hi,int order=0);
//void slow_bounds(SBasis const & s, double &lo, double &hi,int order=0,double tol=0.01);

std::vector<double> roots(SBasis const & s);
std::vector<std::vector<double> > multi_roots(SBasis const &f,
                                 std::vector<double> const &levels,
                                 double tol=1e-7,
                                 double a=0,
                                 double b=1);
    
};

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
#endif

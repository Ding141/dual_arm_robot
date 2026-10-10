#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace ieir_controllers {
// Normalized quintic Hermite segment. Derivatives are in rad/s and rad/s^2.
struct TrajectorySpline {
  double start{0}, end{0};
  std::array<double, 6> c{};
  static TrajectorySpline make(double t0, double t1, double q0, double v0,
    double a0, double q1, double v1, double a1) {
    TrajectorySpline s; s.start=t0; s.end=t1;
    const double t=t1-t0, d=q1-q0, x=v0*t, y=v1*t, z=a0*t*t, w=a1*t*t;
    s.c={q0,x,z/2,10*d-6*x-4*y-1.5*z+.5*w,
      -15*d+8*x+7*y+1.5*z-w,6*d-3*x-3*y-.5*z+.5*w};
    return s;
  }
  void sample(double time, double &q, double &v, double &a) const {
    const double t=end-start, u=std::clamp((time-start)/t,0.,1.);
    q=c[0]+u*(c[1]+u*(c[2]+u*(c[3]+u*(c[4]+u*c[5]))));
    v=(c[1]+u*(2*c[2]+u*(3*c[3]+u*(4*c[4]+u*5*c[5]))))/t;
    a=(2*c[2]+u*(6*c[3]+u*(12*c[4]+u*20*c[5])))/(t*t);
  }
  static double choose(int n,int k) {
    double r=1; for(int i=1;i<=k;++i) r*=double(n-i+1)/i; return r;
  }
  // Bernstein control points bound the entire polynomial, including between ticks.
  bool bounded(double vmax,double amax,double jmax,int depth=0) const {
    const double t=end-start;
    bool ok=true;
    for(int derivative=1;derivative<=3;++derivative) {
      const int n=5-derivative;
      const double limit=derivative==1?vmax:(derivative==2?amax:jmax);
      for(int k=0;k<=n;++k) {
        double b=0;
        for(int i=0;i<=k;++i) {
          double p=c[i+derivative];
          for(int d=1;d<=derivative;++d) p*=double(i+d)/t;
          b+=p*choose(k,i)/choose(n,i);
        }
        if(std::abs(b)>limit+1e-7) ok=false;
      }
    }
    if(ok) return true;
    if(depth>=7) return false;
    double q0,v0,a0,qm,vm,am,q1,v1,a1;
    const double m=(start+end)/2;
    sample(start,q0,v0,a0);sample(m,qm,vm,am);sample(end,q1,v1,a1);
    if(std::abs(vm)>vmax+1e-7 || std::abs(am)>amax+1e-7) return false;
    return make(start,m,q0,v0,a0,qm,vm,am).bounded(vmax,amax,jmax,depth+1) &&
      make(m,end,qm,vm,am,q1,v1,a1).bounded(vmax,amax,jmax,depth+1);
  }
};
}  // namespace ieir_controllers

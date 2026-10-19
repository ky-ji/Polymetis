// Offline 200 Hz quantized input / 1 kHz reference experiment, no hardware.
#include "reference.hpp"
#include <iostream>
#include <vector>
#include <limits>
using R = polymetis_runtime::JointReference;
R::Vector fill(double x) { R::Vector v; v.fill(x); return v; }
void require(bool value, const char* message) { if(!value) throw std::runtime_error(message); }
int main(int argc, char** argv) {
  const double speed = argc == 2 ? std::stod(argv[1]) : .35;
  int baseline_lag[4]{};
  for (double horizon : {0., .02, .04, .06}) {
    for (int kind = 0; kind < 4; ++kind) {
      R r(fill(0),fill(0),fill(-.4),fill(.4),fill(speed),fill(3),fill(40),.001,.15,horizon);
      std::vector<double> input, output;
      double target=0, peak_error=0, max_q=0, max_v=0, max_a=0, max_j=0;
      for (int i=0;i<16000;++i) {
        if (i%5==0) {
          const double t=i*.001;
          if(kind==0) target=.06*std::sin(2*M_PI*.5*t);
          if(kind==1) target=.035*std::sin(2*M_PI*t);
          if(kind==2) target=t<1?0:t<3?.15*(t-1):t<5?.3:t<7?.3-.15*(t-5):0;
          if(kind==3) target=std::clamp(.5*std::sin(2*M_PI*.5*t),-.4,.4);
          target=std::round(target/(2*M_PI/4096))*(2*M_PI/4096);
          target=std::clamp(target,-.4,.4);
        }
        const double prev=r.acceleration[0];
        r.step(fill(target),i/5,static_cast<std::int64_t>(i)*1000000);
        if(r.status) throw std::runtime_error("unexpected fault");
        max_q=std::max(max_q,std::abs(r.q[0]));max_v=std::max(max_v,std::abs(r.velocity[0]));
        max_a=std::max(max_a,std::abs(r.acceleration[0]));max_j=std::max(max_j,std::abs(r.acceleration[0]-prev)/.001);
        if(i>1000) peak_error=std::max(peak_error,std::abs(r.q[0]-target));
        input.push_back(target);output.push_back(r.q[0]);
      }
      double best=1e10; int lag=0;
      for(int shift=0;shift<500;++shift) {
        double error=0;
        for(int i=2000;i<14000;++i) error+=std::pow(output[i+shift]-input[i],2);
        if(error<best) { best=error;lag=shift; }
      }
      std::cout << "h="<<horizon<<" kind="<<kind<<" lag_ms="<<lag<<" peak_error="<<peak_error
                <<" q="<<max_q<<" v="<<max_v<<" a="<<max_a<<" j="<<max_j<<" final="<<r.q[0]<<"\n";
      require(max_q<=.400000001 && max_v<=speed+1e-9 && max_a<=3.000000001 && max_j<=40.000001,
              "streaming prediction escaped the motion envelope");
      if(horizon==0.) baseline_lag[kind]=lag;
      if(horizon==.04 && kind<3) require(lag<baseline_lag[kind]-25,"prediction did not reduce lag");
      if(kind==2) {
        require(std::abs(r.q[0])<1e-9 && std::abs(r.velocity[0])<1e-9,"stationary endpoint did not converge");
        require(max_q<.31,"prediction produced excessive stop overshoot");
      }
    }
  }
  for(double bad : {-1., .061, std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
    bool rejected=false;
    try {R r(fill(0),fill(0),fill(-.4),fill(.4),fill(.35),fill(3),fill(40),.001,.15,bad);}
    catch(const std::invalid_argument&) {rejected=true;}
    require(rejected,"invalid lookahead accepted");
  }
  // Exercise pause, resumed input, deadline and late packets with prediction on.
  R r(fill(0),fill(0),fill(-.4),fill(.4),fill(.35),fill(3),fill(40),.001,.15,.04);
  for(int i=0;i<1000;++i) r.step(fill(i*.0001),i/5,static_cast<std::int64_t>(i)*1000000);
  for(int i=1000;i<3000;++i) r.step(fill(.1),200,static_cast<std::int64_t>(i)*1000000,true);
  require(r.status==0 && std::abs(r.q[0]-.1)<1e-9,"pause retained a predicted offset or expired");
  for(int i=3000;i<4000;++i) r.step(fill(.1-(i-3000)*.0001),201+(i-3000)/5,static_cast<std::int64_t>(i)*1000000);
  for(int i=4000;i<6000;++i) r.step(fill(0),400,static_cast<std::int64_t>(i)*1000000);
  require(r.status==1 && std::abs(r.velocity[0])<1e-9,"stale prediction did not brake and latch");
  auto held=r.q;
  r.step(fill(.2),500,6000000000LL);
  require(r.status==1 && r.q==held,"late packet re-enabled stale predictor");
}

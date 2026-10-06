#include "rl_quadruped_controller/FSM/AbsObservationContract.h"
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

// Non-symmetric fixture mirrored by scripts/test_p1_01_local_contract.py.
// This executable calls the production helper used by StateRL/StateRLRec.
int main() {
  auto f = [](std::initializer_list<float> x) {
    const std::vector<float> values(x);
    return torch::tensor(values).reshape({1, static_cast<long>(values.size())});
  };
  abs_observation::Input x{
    f({.31f,-.27f,.19f}), f({-1,1,1,-1}), f({.11f,-.22f,.37f}), f({.41f,-.52f,-.73f}),
    f({1.2f,-.8f,.65f}), f({.5f}),
    f({3.01f,3.02f,3.03f,1.01f,1.02f,1.03f,4.01f,4.02f,4.03f,2.01f,2.02f,2.03f}),
    f({3,3,3,1,1,1,4,4,4,2,2,2}), f({0,0,0,0,0,0,0,0,0,0,0,0}),
    f({-3,-2,-1,1,2,3,-6,-5,-4,4,5,6}), f({-.91f,-.82f,-.73f,.11f,.22f,.33f,-.66f,-.55f,-.44f,.44f,.55f,.66f}),
    f({-.7f,-.3f,.1f,.2f,.4f,.8f,1.1f,1.7f,2.2f,2.4f,2.58f})};
  const abs_observation::Scale s{1.,1.,.2,100.};
  const auto emit = [](const char* name, const torch::Tensor& t) {
    std::cout << name;
    const auto c=t.flatten().contiguous();
    for (int64_t i=0;i<c.numel();++i) std::cout << (i ? "," : ":") << std::setprecision(9) << c[i].item<float>();
    std::cout << "\n";
  };
  emit("agile", abs_observation::agile(x,s,true));
  emit("ra", abs_observation::ra(x));
  emit("recovery", abs_observation::recovery(x,s,true));
  std::cout << "finite:" << abs_observation::finite(x.ray2d) << "\n";
  x.ray2d[0][0] = std::numeric_limits<float>::quiet_NaN();
  std::cout << "finite_nan:" << abs_observation::finite(x.ray2d) << "\n";
  auto fault = torch::zeros({1,12});
  fault[0][2] = std::numeric_limits<float>::quiet_NaN();
  std::cout << "fault_observation:" << abs_observation::finite(fault) << ",";
  fault.fill_(0); fault[0][4] = std::numeric_limits<float>::quiet_NaN();
  std::cout << abs_observation::finite(fault) << ",";
  fault.fill_(0); fault[0][7] = std::numeric_limits<float>::infinity();
  std::cout << abs_observation::finite(fault) << "\n";
  torch::Tensor previous = torch::zeros({1,4}, torch::kBool);
  const auto c0 = abs_observation::temporalContact(torch::tensor({{true,false,false,false}}), previous);
  const auto c1 = abs_observation::temporalContact(torch::tensor({{false,false,false,false}}), previous);
  const auto c2 = abs_observation::temporalContact(torch::tensor({{false,false,true,false}}), previous);
  emit("contact_touch", c0); emit("contact_liftoff", c1); emit("contact_next", c2);
  std::cout << "timer:" << abs_observation::rollingTimeLeftNormalized(0.,9.) << ","
            << abs_observation::rollingTimeLeftNormalized(4.5,9.) << ","
            << abs_observation::rollingTimeLeftNormalized(9.,9.) << "\n";
  float rays[11] = {0}; const uint64_t now=1000000000ULL;
  std::cout << "ray_valid:" << abs_observation::rayFrameValid(rays,11,0x415253594143544FULL,now,now,200000000ULL) << ","
            << abs_observation::rayFrameValid(rays,11,0,now,now,200000000ULL) << ","
            << abs_observation::rayFrameValid(rays,11,0x415253594143544FULL,now-300000000ULL,now,200000000ULL) << ",";
  rays[4] = std::numeric_limits<float>::infinity();
  std::cout << abs_observation::rayFrameValid(rays,11,0x415253594143544FULL,now,now,200000000ULL) << "\n";
}

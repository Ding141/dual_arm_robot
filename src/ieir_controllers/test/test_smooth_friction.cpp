#include <gtest/gtest.h>
#include "ieir_controllers/smooth_friction.hpp"
using namespace ieir_controllers;
TEST(SmoothFriction, ZeroAndDirections) {
  EXPECT_EQ(smooth_friction(0,.01,.45,.35,.01,.8,.8),0);
  EXPECT_NEAR(smooth_friction(.1,.01,.45,.35,.01,.8,.8),.3608,1e-8);
  EXPECT_NEAR(smooth_friction(-.1,.01,.45,.35,.01,.8,.8),-.2808,1e-8);
  EXPECT_LT(std::abs(smooth_friction(1e-9,.01,.45,.35,.01,.8,.8)),1e-7);
  EXPECT_LT(std::abs(smooth_friction(-1e-9,.01,.45,.35,.01,.8,.8)),1e-7);
}
TEST(SmoothFriction, BoundedAndInvalidVelocity) {
  EXPECT_EQ(smooth_friction(100,.01,2,2,1,1,.8),.8);
  EXPECT_EQ(smooth_friction(-100,.01,2,2,1,1,.8),-.8);
  EXPECT_EQ(smooth_friction(NAN,.01,2,2,1,1,.8),0);
}
TEST(SmoothFriction, NoOvershootAndLateCycleBound) {
  EXPECT_NEAR(friction_slew(0,.8,8,1./300),8./300,1e-12);
  EXPECT_EQ(friction_slew(.79,.8,8,1./300),.8);
  EXPECT_NEAR(friction_slew(.3,-.3,8,10),.22,1e-12);
  EXPECT_EQ(friction_slew(.3,-.3,8,-1),.3);
  EXPECT_NEAR(friction_slew(.3,0,8,1./300),.3-8./300,1e-12);
}

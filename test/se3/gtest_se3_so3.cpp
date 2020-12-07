#include <gtest/gtest.h>

#include "manif/manif.h"

#include "../test_utils.h"

using namespace manif;

TEST(TEST_SO3EXPLOG, TEST_EXPLOG)
{
  SO3Tangentd delta = SO3Tangentd::Random();

  SO3d state = delta.exp();
  SO3Tangentd delta_other = state.log();

  EXPECT_MANIF_NEAR(delta, delta_other, 1e-9);
}

TEST(TEST_SE3EXPLOG, TEST_EXPLOG)
{
  SE3Tangentd delta = SE3Tangentd::Random();

  SE3d state = delta.exp();
  SE3Tangentd delta_other = state.log();

  EXPECT_MANIF_NEAR(delta, delta_other, 1e-9);
}

int main(int argc, char** argv)
{
  std::srand((unsigned int) time(0));

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

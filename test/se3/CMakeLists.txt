# SE3 tests

manif_add_gtest(gtest_se3 gtest_se3.cpp)

manif_add_gtest(gtest_se3debug gtest_se3_so3.cpp)

set(CXX_11_TEST_TARGETS

  ${CXX_11_TEST_TARGETS}

  # SE3
  gtest_se3
  gtest_se3debug

  PARENT_SCOPE
)

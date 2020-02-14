#include <gtest/gtest.h>
#include <ros/ros.h>

#include "kermit/kernel/Kernel.hpp"

/* Gtest
 * For a primer, visit:
 * https://github.com/google/googletest/blob/master/googletest/docs/primer.md
 */

class HelloWorld {};

TEST(HelloWorld, BasicTest) { ASSERT_EQ(1, 1); }

TEST(KernelTest, InvalidTeensy) {
        Kernel k{};
        // I feel like this should return false
        ASSERT_TRUE(k.initialize_teensy("aaaaaa"));
}

int main(int argc, char *argv[]) {
        testing::InitGoogleTest(&argc, argv);
        ros::init(argc, argv, "tester");
        ros::NodeHandle nh;
        return RUN_ALL_TESTS();
}
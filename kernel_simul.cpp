/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : kernel
 * @created     : Tuesday Jan 07, 2020 13:43:04 MST
 */

#include "kermit/kernel/Kernel.hpp"

int main(int argc, char **argv) {

        ros::init(argc, argv, nodes::Kernel_Node);

        Kernel kernel(false, true);
        ros::spin();
        return 0;
}

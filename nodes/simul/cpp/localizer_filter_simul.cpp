/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : localizer_filter
 * @created     : Tuesday Jan 07, 2020 17:53:12 MST
 */

#include "kermit/localizer_filter/LocalizerFilter.hpp"
#include "kermit/common.h"

int main(int argc, char **argv) {

        ros::init(argc, argv, nodes::Localizer_Node);

        LocalizerFilter filter;

        ros::spin();
        return 0;
}

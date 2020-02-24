//
// Created by anyone on 2/03/17.
//

#ifndef MARKER_PARAMETERS_H
#define MARKER_PARAMETERS_H

#include <ros/package.h>

namespace cares {
    namespace marker{
        const std::string IMAGE_S       = "image";
        const std::string MARKERS_S     = "markers";
        const std::string DEPTH_IMAGE_S = "depth_image";
        const std::string CAMERA_INFO_S = "camera_info";
        const std::string MARKER_SIZE_D = "marker_size";
        const std::string TF_PREFIX_S   = "tf_prefix";
        const std::string DISPLAY_B     = "display";
    }
}

#endif //MARKER_PARAMETERS_H

//
// Created by anyone on 2/03/17.
//

#ifndef MARKER_PARAMETERS_H
#define MARKER_PARAMETERS_H

#include <ros/package.h>

namespace cares {
    namespace marker{
        const std::string IMAGE_S       = "image";
        const std::string IMAGE_LEFT_S  = "image_left";
        const std::string IMAGE_RIGHT_S = "image_right";
        const std::string STEREO_INFO_S = "stereo_info";
        const std::string MARKERS_S     = "markers";
        const std::string DEPTH_IMAGE_S = "depth_image";
        const std::string CAMERA_INFO_S = "camera_info";
        const std::string MARKER_SIZE_D = "marker_size";
        const std::string TF_PREFIX_S   = "tf_prefix";
        const std::string DISPLAY_B     = "display";
        const std::string IS_DEPTH_IN_METERS = "is_depth_in_meters";

        const std::string DICTIONARY_I    = "dictionary";

        const std::string CENTRE_I    = "centre";
        const std::string TOP_LEFT_I  = "top_left";
        const std::string TOP_RIGHT_I = "top_right";
        const std::string BOT_RIGHT_I = "bot_right";
        const std::string BOT_LEFT_I  = "bot_left";

    }
}

#endif //MARKER_PARAMETERS_H

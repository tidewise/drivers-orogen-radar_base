#ifndef radar_base_TYPES_HPP
#define radar_base_TYPES_HPP

#include <base/Time.hpp>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace radar_base {

    struct RadarFrameExportConfig {
        base::Time time_between_frames;
        bool use_heading_correction = true;
        int window_size = 1024;
        float beam_width = 0.0174;
    };
}

#endif

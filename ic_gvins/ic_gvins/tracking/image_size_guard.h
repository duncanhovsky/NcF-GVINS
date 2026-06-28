#ifndef GVINS_TRACKING_IMAGE_SIZE_GUARD_H
#define GVINS_TRACKING_IMAGE_SIZE_GUARD_H

namespace tracking_utils {

inline bool isImageSizeCompatible(int image_cols, int image_rows, int camera_width, int camera_height) {
    return image_cols > 0 && image_rows > 0 && camera_width > 0 && camera_height > 0 &&
           image_cols == camera_width && image_rows == camera_height;
}

} // namespace tracking_utils

#endif // GVINS_TRACKING_IMAGE_SIZE_GUARD_H

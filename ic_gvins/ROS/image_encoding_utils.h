#ifndef GVINS_ROS_IMAGE_ENCODING_UTILS_H
#define GVINS_ROS_IMAGE_ENCODING_UTILS_H

#include <string>

namespace image_encoding_utils {

enum class EncodingKind {
    MONO8,
    BGR8,
    RGB8,
    BAYER_BGGR8,
    BAYER_GBRG8,
    BAYER_GRBG8,
    BAYER_RGGB8,
    UNSUPPORTED,
};

inline EncodingKind classifyEncoding(const std::string &encoding) {
    if (encoding == "mono8") {
        return EncodingKind::MONO8;
    }
    if (encoding == "bgr8") {
        return EncodingKind::BGR8;
    }
    if (encoding == "rgb8") {
        return EncodingKind::RGB8;
    }
    if (encoding == "bayer_bggr8") {
        return EncodingKind::BAYER_BGGR8;
    }
    if (encoding == "bayer_gbrg8") {
        return EncodingKind::BAYER_GBRG8;
    }
    if (encoding == "bayer_grbg8") {
        return EncodingKind::BAYER_GRBG8;
    }
    if (encoding == "bayer_rggb8") {
        return EncodingKind::BAYER_RGGB8;
    }
    return EncodingKind::UNSUPPORTED;
}

inline int bytesPerPixel(EncodingKind encoding) {
    switch (encoding) {
    case EncodingKind::MONO8:
    case EncodingKind::BAYER_BGGR8:
    case EncodingKind::BAYER_GBRG8:
    case EncodingKind::BAYER_GRBG8:
    case EncodingKind::BAYER_RGGB8:
        return 1;
    case EncodingKind::BGR8:
    case EncodingKind::RGB8:
        return 3;
    case EncodingKind::UNSUPPORTED:
        return 0;
    }
    return 0;
}

} // namespace image_encoding_utils

#endif // GVINS_ROS_IMAGE_ENCODING_UTILS_H

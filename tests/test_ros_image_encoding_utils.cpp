#include "ROS/image_encoding_utils.h"

#include <cassert>

int main() {
    using image_encoding_utils::EncodingKind;

    assert(image_encoding_utils::classifyEncoding("mono8") == EncodingKind::MONO8);
    assert(image_encoding_utils::classifyEncoding("bgr8") == EncodingKind::BGR8);
    assert(image_encoding_utils::classifyEncoding("rgb8") == EncodingKind::RGB8);
    assert(image_encoding_utils::classifyEncoding("bayer_bggr8") == EncodingKind::BAYER_BGGR8);
    assert(image_encoding_utils::classifyEncoding("bayer_gbrg8") == EncodingKind::BAYER_GBRG8);
    assert(image_encoding_utils::classifyEncoding("bayer_grbg8") == EncodingKind::BAYER_GRBG8);
    assert(image_encoding_utils::classifyEncoding("bayer_rggb8") == EncodingKind::BAYER_RGGB8);
    assert(image_encoding_utils::classifyEncoding("yuv422") == EncodingKind::UNSUPPORTED);

    assert(image_encoding_utils::bytesPerPixel(EncodingKind::MONO8) == 1);
    assert(image_encoding_utils::bytesPerPixel(EncodingKind::BAYER_BGGR8) == 1);
    assert(image_encoding_utils::bytesPerPixel(EncodingKind::BGR8) == 3);
    assert(image_encoding_utils::bytesPerPixel(EncodingKind::UNSUPPORTED) == 0);

    return 0;
}

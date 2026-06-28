#include "tracking/image_size_guard.h"

#include <cassert>

int main() {
    assert(tracking_utils::isImageSizeCompatible(1280, 560, 1280, 560));
    assert(!tracking_utils::isImageSizeCompatible(640, 560, 1280, 560));
    assert(!tracking_utils::isImageSizeCompatible(1280, 480, 1280, 560));
    assert(!tracking_utils::isImageSizeCompatible(0, 560, 1280, 560));
    assert(!tracking_utils::isImageSizeCompatible(1280, 0, 1280, 560));

    return 0;
}

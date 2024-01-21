#include "primitive.h"

std::vector<UPS::PrimitivePtr> UPS::Primitive::primitives;

UPS::TimeObject UPS::TimeObject::operator()(Primitive* p) const {
    auto tmp = *this;
    tmp.inner_time = p->getInnerTime();
    tmp.relative_frame_number = p->relativeSlideIndex(tmp.absolute_frame_number);
    return tmp;
}

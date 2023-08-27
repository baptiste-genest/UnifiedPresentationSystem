#ifndef TIMEOBJECT_H
#define TIMEOBJECT_H
#include "../UPS.h"

namespace UPS {

struct TimeObject
{
    TimeTypeSec from_begin;
    TimeTypeSec from_action;
    int absolute_frame_number;
};

}

#endif // TIMEOBJECT_H

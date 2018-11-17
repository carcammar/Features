#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "frame.h"

class KeyFrame: public Frame
{
public:
    KeyFrame();
    KeyFrame(Frame *pF, unsigned int Id);
private:
    Frame *mpF; // Frame where this KF is created from
};

#endif // KEYFRAME_H

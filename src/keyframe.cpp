#include "keyframe.h"

KeyFrame::KeyFrame()
{

}

KeyFrame::KeyFrame(Frame *pF, unsigned int Id): mpF(pF)
{
    mId = Id;
    mTs = pF->mTs;

    mmpPIdx = pF->GetpPIdx();
    mmpPObs = pF->GetpPObs();

    mK = pF->mK;
    mcvK = pF->mcvK.clone();
    fx = pF->fx;
    fy = pF->fy;
    cx = pF->cx;
    cy = pF->cy;
    heigh = pF->heigh;
    width = pF->width;

    // Map information
    mpMap = pF->GetMap();
    mpLocalMap = pF->GetLocalMap();

    mvDistCoeffs = pF->mvDistCoeffs;
    mTcw = pF->GetPose();

    mpFeatExt = pF->mpFeatExt;
    mvKP = pF->GetKPs();
    mvUndPoints = pF->GetUndPoints();
    mvDistPoints = pF->GetDistPoints();
    mvpP = pF->GetObservedPoints();
    mDesc = pF->GetDescriptors();
    // TODO terminar constructor...
}

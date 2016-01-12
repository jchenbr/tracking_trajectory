
#ifndef _TRACKING_TRAJECTORY_MACRO_H_
#define _TRACKING_TRAJECTORY_MACRO_H_

namespace TrackingTrajectory
{
    const int _BEG = 0;
    const int _ROW_BEG = 0;
    const int _COL_BEG = 0;
    const int _DIM_BEG = 0;
    const int _BDY_BEG = 0;

    const int _DIM_NUM = 3;
    const int _DIM_X = 0;
    const int _DIM_Y = 1;
    const int _DIM_Z = 2;

    const int _BDY_NUM = 6;
    const int _BDY_L_X = 0;
    const int _BDY_R_X = 1;
    const int _BDY_L_Y = 2;
    const int _BDY_R_Y = 3;
    const int _BDY_L_Z = 4;
    const int _BDY_R_Z = 5;
    const int _BDY_L_T = 6;
    const int _BDY_R_T = 7;

    const int _TOT_BDY = 6;
    const int _TOT_DIM = 3;
    const int _TOT_STT = 3;

    const int _STT_POS = 0;
    const int _STT_VEL = 1;
    const int _STT_ACC = 2;

    const int _EPS = 1e-9;
}
#endif

#ifndef __CAMERA_PARAMS_H__
#define __CAMERA_PARAMS_H__

#include <stdint.h>

//choose your camera
#define ZENIT_ET
//#define BRONICA_ETRSI

#if defined(ZENIT_ET)
#include "zenit_et.h"
#elif defined(BRONICA_ETRSI)
#include "bronica_etrsi.h"
#endif

#endif

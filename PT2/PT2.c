/*
 * MIT License
 * 
 * Copyright (c) 2019 Slaven Glumac
 * 
 * Proportional delay element:
 * T1 Ts y''(t) + (T1 + Ts)y(t) = Ku(t)
 */
#include <fmi2Functions.h>
#include <math.h>

#define MAX_INPUT_DERIVATIVE_ORDER 0
#define NUMBER_OF_REALS 7
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2, 3, 4, 5, 6};

#define vr_u 0
#define _u(d) r(vr_u,d)
#define _y r(1,0)
#define _K r(2,0)
#define _T1 r(3,0)
#define _Ts r(4,0)
#define _x10 r(5,0)
#define _x20 r(6,0)

struct Internal
{
    fmi2Real x1;
	fmi2Real x2;
};

#include <template.h>

#define _x1 (_internal.x1)
#define _x2 (_internal.x2)

void InstantiateInternal(fmi2Component component)
{
}

void FreeInternal(fmi2Component component)
{
}

void StartInitialization(fmi2Component component)
{
    _u(0) = 0.;
    _K = 1.;
    _T1 = 5.;
	_Ts = 1.;
}

fmi2Status FinishInitialization(fmi2Component component)
{
    _x1 = _x10;
	_x2 = _x20;
    _y = _x2;
    return fmi2OK;
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    fmi2Real x1 = _x1;
	fmi2Real x2 = _x2;
	fmi2Real u = _K * _u(0);
	
	_x1 = u * h / (_T1 + h) + x1 * _T1 / (_T1 + h);
	_x2 = x1 * h / (_Ts + h) + x2 * _Ts / (_Ts + h);
	
	
	_y = _x2;

    logf(fmi2OK, "h = %lf, u = %lf, x1 = %lf, x2 = %lf, y = %lf", h,  _u(0), _x1, _x2, _y);
    return fmi2OK;
}

fmi2Status OutputUpdate(fmi2Component component)
{
	return fmi2OK;
}

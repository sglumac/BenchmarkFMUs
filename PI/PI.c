/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 * 
 * The code for PI controller with reference given as a fixed parameter r:
 * e(t) = r - u(t)
 * x'(t) = e(t)
 * y(t) = KP * e(t) + KI * x(t)
 */
#include <fmi2Functions.h>
#include <math.h>

#define MAX_INPUT_DERIVATIVE_ORDER 10
#define NUMBER_OF_REALS 6
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2, 3, 4, 5};

struct Internal
{
    fmi2Real x;
};

#include <template.h>

#define vr_u 0
#define _u(d) r(vr_u,d)
#define _y r(1,0)
#define _KP r(2,0)
#define _KI r(3,0)
#define _r r(4,0)
#define _x0 r(5,0)

#define _x (_internal.x)

void InstantiateInternal(fmi2Component component)
{
}

void FreeInternal(fmi2Component component)
{
}

void StartInitialization(fmi2Component component)
{
    _u(0) = 0.;
    _KI = 1.;
    _KP = 1.;
    _r = 1.;
    logf(fmi2OK, "u = %lf, KI = %lf, KP = %lf, r = %lf", _u(0), _KI, _KP, _r);
}

fmi2Status FinishInitialization(fmi2Component component)
{
    fmi2Real e = _r - _u(0);
    _x = _x0;
    _y = _KI * _x + _KP * e;
    return fmi2OK;
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    size_t d;
    fmi2Real hn = 1.;
    fmi2Real u = interp(component, vr_u, h);
    fmi2Real e = _r - u;
    for (d = 0; d < MAX_INPUT_DERIVATIVE_ORDER; d++)
    {
        hn *= h;
        _x -= _u(d) * hn / (d + 1);
    }
    _x += _r * h;
    _y = _KI * _x + _KP * e;
    logf(fmi2OK, "r = %lf, u = %lf, u0 = %lf,  e = %lf, x = %lf", _r, u, _u(0), e, _x);
    return fmi2OK;
}


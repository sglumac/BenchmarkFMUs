/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 * 
 * Proportional delay element:
 * Ty'(t) + y(t) = Ku(t)
 */
#include <fmi2Functions.h>
#include <math.h>

#define MAX_INPUT_DERIVATIVE_ORDER 10
#define NUMBER_OF_REALS 5
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2, 3, 4};

#define vr_u 0
#define _u(d) r(vr_u,d)
#define _y r(1,0)
#define _K r(2,0)
#define _T r(3,0)
#define _x0 r(4,0)

struct Internal
{
    fmi2Real x;
};

#include <template.h>

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
    _K = 1.;
    _T = 1.;
}

fmi2Status FinishInitialization(fmi2Component component)
{
    _x = _x0;
    _y = _x;
    return fmi2OK;
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    fmi2Real x0 = _x;
    fmi2Real emhT = exp(-h / _T);
    fmi2Real C;
    fmi2Real hk = 1.;
    fmi2Real a[MAX_INPUT_DERIVATIVE_ORDER+1];
    size_t k;

    a[MAX_INPUT_DERIVATIVE_ORDER] = _K * _u(MAX_INPUT_DERIVATIVE_ORDER);
    for (k = MAX_INPUT_DERIVATIVE_ORDER; k > 0; k--)
    {
        a[k-1] = _K * _u(k-1) - _T * k * a[k];
    }
    C = x0 - a[0];

    _x = C * emhT;
    for (k = 0; k <= MAX_INPUT_DERIVATIVE_ORDER; k++)
    {
        logf(fmi2OK, "x = %lf", _x);
        _x += a[k] * hk;
        hk *= h;
    }
    _y = _x;
    logf(fmi2OK, "h = %lf, e(-h / T) = %lf, u0 = %lf, u1 = %lf, x0 = %lf, C = %lf, a0 = %lf, a1 = %lf, x = %lf", h, emhT, _u(0), _u(1), x0, C, a[0], a[1], _x);
    return fmi2OK;
}


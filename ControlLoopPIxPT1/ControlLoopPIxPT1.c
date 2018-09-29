/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 * 
 * Control loop reference model with PI controller and PT1 plant.
 */
#include <fmi2Functions.h>
#include <math.h>

#define MAX_INPUT_DERIVATIVE_ORDER 0
#define NUMBER_OF_REALS 9
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8};

struct Internal
{
    fmi2Real x_PI;
    fmi2Real x_PT1;
};

#include <template.h>

#define _u r(0,0)
#define _y r(1,0)
#define _KP r(2,0)
#define _KI r(3,0)
#define _K r(4,0)
#define _T r(5,0)
#define _r r(6,0)
#define _x0_PI r(7,0)
#define _x0_PT1 r(8,0)

#define _x_PI (_internal.x_PI)
#define _x_PT1 (_internal.x_PT1)

void InstantiateInternal(fmi2Component component)
{
}

void FreeInternal(fmi2Component component)
{
}

void StartInitialization(fmi2Component component)
{
    _KP = 1.;
    _KI = 1.;
    _K = 1.;
    _T = 1.;
    _r = 1.;
    _x0_PI = 0.;
    _x0_PT1 = 0.;
    logf(fmi2OK, "KP = %lf, KI = %lf, K = %lf, T = %lf, r = %lf", _KP, _KI, _K, _T, _r);
}

fmi2Status FinishInitialization(fmi2Component component)
{
    fmi2Real e;
    _x_PI = _x0_PI;
    _x_PT1 = _x0_PT1;
    _y = _x_PT1;
    e = _r - _y;
    _u = _KP * e + _KI * _x_PI;
    logf(fmi2OK, "u = %lf, y = %lf", _u, _y);

    return fmi2OK;
}

fmi2Real texponential(fmi2Real x0, fmi2Real v0, fmi2Real C0, fmi2Real b, fmi2Real t)
{
    fmi2Real C1 = x0 - C0;
    fmi2Real C2 = v0 + b / 2. * C1;
    return C0 + C1 * exp(-b / 2. * t) + C2 * t * exp(-b / 2. * t);
}

fmi2Real damped_sine(fmi2Real x0, fmi2Real v0, fmi2Real C0, fmi2Real b, fmi2Real c, fmi2Real t)
{
    fmi2Real omega = sqrt(c - b * b / 4.);
    fmi2Real C2 = x0 - C0;
    fmi2Real C1 = (v0 + b / 2. * C2) / omega;
    fmi2Real emb2t = exp(-b / 2. * t);
    return C0 + C1 * emb2t * sin(omega * t) + C2 * emb2t * cos(omega * t);
}

fmi2Real exponential(fmi2Real x0, fmi2Real v0, fmi2Real C0, fmi2Real b, fmi2Real c, fmi2Real t)
{
    fmi2Real sq = sqrt(b * b / 4. - c);
    fmi2Real lambda1 = -b / 2. - sq;
    fmi2Real lambda2 = -b / 2. + sq;
    fmi2Real C1 = (v0 + C0 * lambda2 - lambda2 * x0) / (lambda1 - lambda2);
    fmi2Real C2 = x0 - C0 - C1;
    return C0 + C1 * exp(lambda1 * t) + C2 * exp(lambda2 * t);
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    fmi2Real x10 = _x_PI;
    fmi2Real x20 = _x_PT1;
    fmi2Real v10 = _r - x20;
    fmi2Real v20 = _K * _KI * x10 / _T - (1 + _K * _KP) * x20 / _T + _K * _KP * _r / _T;
    fmi2Real C01 = _r / (_K * _KI);
    fmi2Real C02 = _r;

    fmi2Real b =  (1. + _K * _KP) / _T;
    fmi2Real c = _K * _KI / _T;

    log(fmi2OK, "StatuUpdate Begin");

    if (fabs(b * b / 4. - c) < 1e-8)
    {
        _x_PI = texponential(x10, v10, C01, b, h);
        _x_PT1 = texponential(x20, v20, C02, b, h);
    }
    else if (c > b * b / 4.)
    {
        _x_PI = damped_sine(x10, v10, C01, b, c, h);
        _x_PT1 = damped_sine(x20, v20, C02, b, c, h);
    }
    else
    {
        _x_PI = exponential(x10, v10, C01, b, c, h);
        _x_PT1 = exponential(x20, v20, C02, b, c, h);
    }

    _y = _x_PT1;
    _u = _KP * (_r - _y) + _KI * _x_PI;
    logf(fmi2OK, "u = %lf, y = %lf", _u, _y);

    return fmi2OK;
}

fmi2Status OutputUpdate(fmi2Component component)
{
	return fmi2OK;
}


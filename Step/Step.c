/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 * 
 * The code for step signal:
 * y(t) = { y0, t < tStep
 *        { yStep, t >= tStep
 * x'(t) = e(t)
 * y(t) = KP * e(t) + KI * x(t)
 */
#include <fmi2Functions.h>
#include <math.h>

#define MAX_INPUT_DERIVATIVE_ORDER 10
#define NUMBER_OF_REALS 4
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2, 3};

struct Internal
{
    fmi2Real x;
};

#include <template.h>

#define _y r(0, 0)
#define _y0 r(1,0)
#define _yEnd r(2,0)
#define _tStep r(3,0)

#define _x (_internal.x)

void InstantiateInternal(fmi2Component component)
{
}

void FreeInternal(fmi2Component component)
{
}

void StartInitialization(fmi2Component component)
{
    _y = 0.;
    _y0 = 1.;
    _yEnd = 1.;
    _tStep = 1.;
    logf(fmi2OK, "y = %lf, y0 = %lf, yEnd = %lf, _tStep = %lf", _y, _y0, _yEnd, _tStep);
}

fmi2Status FinishInitialization(fmi2Component component)
{
    _y = _t < _tStep ? _y0 : _yEnd;
    return fmi2OK;
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    _y = _t + h < _tStep ? _y0 : _yEnd;
    logf(fmi2OK, "y = %lf, y0 = %lf, yEnd = %lf, _tStep = %lf", _y, _y0, _yEnd, _tStep);
    return fmi2OK;
}


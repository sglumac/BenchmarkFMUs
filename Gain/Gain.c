/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 * 
 * Gain element:
 * y(t) = Ku(t)
 */
#include <fmi2Functions.h>
#include <math.h>

#define MAX_INPUT_DERIVATIVE_ORDER 10
#define NUMBER_OF_REALS 3
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2};

#define vr_u 0
#define _u r(0,0)
#define _y r(1,0)
#define _K r(2,0)

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
    _u = 0.;
    _K = 1.;
}

fmi2Status FinishInitialization(fmi2Component component)
{
    _y = _K * _u;
    return fmi2OK;
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    logf(fmi2OK, "u = %lf, y = %lf, K = %lf", _u, _y, _K);
    return fmi2OK;
}

fmi2Status OutputUpdate(fmi2Component component)
{
	_y = _K * _u;
	return fmi2OK;
}
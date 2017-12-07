/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 * 
 * The code for Subtraction FMU:
 * y(t) = u1(t) - u2(t)

 */
#include <fmi2Functions.h>
#include <math.h>

#define MAX_INPUT_DERIVATIVE_ORDER 10
#define NUMBER_OF_REALS 3
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2};

struct Internal
{
    fmi2Real x;
};

#include <template.h>

#define _u1 r(0,0)
#define _u2 r(1,0)
#define _y r(2,0)

#define _x (_internal.x)

void InstantiateInternal(fmi2Component component)
{
}

void FreeInternal(fmi2Component component)
{
}

void StartInitialization(fmi2Component component)
{
    _u1 = 0.;
    _u2 = 0.;
    _y = 0.;
}

fmi2Status FinishInitialization(fmi2Component component)
{
    _y = _u1 - _u2;
    return fmi2OK;
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    _y = _u1 - _u2;
    logf(fmi2OK, "u1 = %lf, u2 = %lf, y = %lf", _u1, _u2, _y);
    return fmi2OK;
}

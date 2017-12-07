/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 * 
 * The code for zero signal:
 * y(t) = 0
 */
#include <fmi2Functions.h>
#include <math.h>

#define MAX_INPUT_DERIVATIVE_ORDER 10
#define NUMBER_OF_REALS 1
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0};

struct Internal
{
    fmi2Real x;
};

#include <template.h>

#define _y r(0, 0)

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
}

fmi2Status FinishInitialization(fmi2Component component)
{
    return fmi2OK;
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    return fmi2OK;
}


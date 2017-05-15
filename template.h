/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 *
 * Template for FMUs which contains boiler-plate code for FMI compliant binary.
 * It allows for input interpolation up to MAX_INPUT_DERIVATIVE_ORDER
 *
 * Code using this should define the following macros:
 * MAX_INPUT_DERIVATIVE_ORDER, NUMBER_OF_REALS, NUMBER_OF_INTEGERS,
 * NUMBER_OF_BOOLEANS, NUMBER_OF_STRINGS
 * initalize the following mapping:
 * const fmi2ValueReference ivrs[]
 * and define the following struct:
 * struct Internal
 * 
 * The following functions should be defined:
 * void InstantiateInternal(fmi2Component component);
 * void FreeInternal(fmi2Component component);
 * void StartInitialization(fmi2Component component);
 * fmi2Status FinishInitialization(fmi2Component component);
 * fmi2Status StateUpdate(fmi2Component component, fmi2Real communicationStepSize);
 */
#ifndef TEMPLATE_H
#define TEMPLATE_H
#include <fmi2Functions.h>
#include <string.h>

#define _this ((struct Component*)component)

#define r(vr,d) _this->reals[ivrs[vr]][d]
#define i(vr) _this->integers[ivrs[vr]]
#define b(vr) _this->booleans[ivrs[vr]]
#define s(vr) _this->strings[ivrs[vr]]

#define _t _this->time
#define _tolerance _this->tolerance
#define _internal (_this->internal)
#include <stdio.h>
#define log(status, message)\
    if (_this->loggingOn) \
    { \
        _this->callbacks->logger \
            ( _this->callbacks->componentEnvironment \
            , _this->instanceName \
            , status \
            , "logAll" \
            , message); \
    }
#define logf(status, message, ...)\
    if (_this->loggingOn) \
    { \
        _this->callbacks->logger \
            ( _this->callbacks->componentEnvironment \
            , _this->instanceName \
            , status \
            , "logAll" \
            , message, ##__VA_ARGS__); \
    }

void InstantiateInternal(fmi2Component component);
void FreeInternal(fmi2Component component);
void StartInitialization(fmi2Component component);
fmi2Status FinishInitialization(fmi2Component component);
fmi2Status StateUpdate(fmi2Component component, fmi2Real communicationStepSize);

struct Component
{
    fmi2Real** reals;
    fmi2Integer* integers;
    fmi2Boolean* booleans;
    fmi2String* strings;
    fmi2Char* instanceName;
    fmi2Real time;
    fmi2Real startTime;
    fmi2Real stopTime;
    fmi2Real tolerance;
    fmi2Boolean loggingOn;
    const fmi2CallbackFunctions* callbacks;
    struct Internal internal;
};

fmi2Real interp(fmi2Component component, fmi2ValueReference vr, fmi2Real dt)
{
    fmi2Real u = r(vr,0);
#if MAX_INPUT_DERIVATIVE_ORDER > 0
    size_t d, fact;
    for (d = 1; d < MAX_INPUT_DERIVATIVE_ORDER; d++)
    {
        fmi2Real x = r(vr, d);
        for (fact = 1; fact <= d; fact++)
        {
            x *= dt / fact;
        }
        u += x;
    }
#endif
    return u;
}

const char* fmi2GetTypesPlatform(void)
{
    return fmi2TypesPlatform;
}

const char* fmi2GetVersion(void)
{
    return "2.0";
}

fmi2Status fmi2SetDebugLogging
    ( fmi2Component component
    , fmi2Boolean loggingOn
    , size_t nCategories
    , const fmi2String categories[])
{
    struct Component* c = component;
    if (c == NULL)
    {
        return fmi2Fatal;
    }
    c->loggingOn = loggingOn;
    return fmi2OK;
}

fmi2Status fmi2Terminate(fmi2Component component)
{
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    log(fmi2OK, "fmi2Terminate");
    return fmi2OK;
}

fmi2Status fmi2Reset(fmi2Component component)
{
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    log(fmi2OK, "fmi2Reset");
    return fmi2OK;
}

fmi2Component fmi2Instantiate
    ( fmi2String instanceName
    , fmi2Type fmuType
    , fmi2String fmuGUID
    , fmi2String fmuResourceLocation
    , const fmi2CallbackFunctions* callbacks
    , fmi2Boolean visible
    , fmi2Boolean loggingOn)
{
    struct Component* c = callbacks->allocateMemory(1, sizeof(struct Component));
    size_t i, j;
    c->reals = callbacks->allocateMemory(NUMBER_OF_REALS, sizeof(fmi2Real*));
    for (i = 0; i < NUMBER_OF_REALS; i++)
    {
        c->reals[i] = callbacks->allocateMemory(MAX_INPUT_DERIVATIVE_ORDER+1, sizeof(fmi2Real));
        for (j = 0; j <= MAX_INPUT_DERIVATIVE_ORDER; j++)
        {
            c->reals[i][j] = 0;
        }
    }
    c->integers = callbacks->allocateMemory(NUMBER_OF_INTEGERS, sizeof(fmi2Real));
    c->booleans = callbacks->allocateMemory(NUMBER_OF_BOOLEANS, sizeof(fmi2Real));
    c->strings = callbacks->allocateMemory(NUMBER_OF_STRINGS, sizeof(fmi2Real));
    c->callbacks = callbacks;
    c->instanceName = callbacks->allocateMemory(1 + strlen(instanceName), sizeof(fmi2Char));
    strcpy(c->instanceName, instanceName);
    if (callbacks->logger == NULL)
    {
        c->loggingOn = fmi2False;
    }
    else
    {
        c->loggingOn = loggingOn;
    }
    InstantiateInternal(c);
    return c;
}

void fmi2FreeInstance(fmi2Component component)
{
    struct Component* c = component;
    size_t i;
    if (component == NULL)
    {
        return;
    }
    FreeInternal(c);
    c->callbacks->freeMemory(c->instanceName);
    for (i = 0; i < NUMBER_OF_REALS; i++)
    {
        c->callbacks->freeMemory(c->reals[i]);
    }
    c->callbacks->freeMemory(c->reals);
    c->callbacks->freeMemory(c->integers);
    c->callbacks->freeMemory(c->booleans);
    c->callbacks->freeMemory(c->strings);
    c->callbacks->freeMemory(component);
}

fmi2Status fmi2SetupExperiment
    ( fmi2Component component
    , fmi2Boolean toleranceDefined
    , fmi2Real tolerance
    , fmi2Real startTime
    , fmi2Boolean stopTimeDefined
    , fmi2Real stopTime)
{
    struct Component* c = component; 
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    c->startTime = startTime;
    logf(fmi2OK, "fmi2SetupExperiment startTime = %lf", startTime);
    c->time = startTime;
    if (stopTimeDefined)
    {
        if (stopTime < 0.)
        {
            return fmi2Error;
        }
        c->stopTime = stopTime;
        logf(fmi2OK, "stopTime = %lf", stopTime);
    }
    if (toleranceDefined)
    {
        c->tolerance = tolerance;
        logf(fmi2OK, "tolerance = %lf", tolerance);
    }
    else
    {
        c->tolerance = 1e-3;
    }
    return fmi2OK; 
}

fmi2Status fmi2EnterInitializationMode(fmi2Component component)
{
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    log(fmi2OK, "fmi2EnterInitializationMode");
    StartInitialization(component);
    return fmi2OK;
}

fmi2Status fmi2ExitInitializationMode(fmi2Component component)
{
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    log(fmi2OK, "fmi2ExitInitializationMode");
    return FinishInitialization(component);
}

fmi2Status fmi2DoStep
  ( fmi2Component component
  , fmi2Real currentCommunicationPoint
  , fmi2Real communicationStepSize
  , fmi2Boolean noSetFMUStatePriorToCurrentPoint)
{
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    if (StateUpdate(component, communicationStepSize) != fmi2OK)
    {
        return fmi2Error;
    }
    _t += communicationStepSize;
    return fmi2OK;
}

fmi2Status fmi2GetReal
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , fmi2Real value[])
{
    size_t i;
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    for (i = 0; i < nvr; i++)
    {
        value[i] = r(vr[i],0);
        logf(fmi2OK, "fmi2GetReal vr = %d, value = %lf", vr[i], value[i]);
    }
    return fmi2OK;
}

fmi2Status fmi2GetInteger
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , fmi2Integer value[])
{
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    size_t i;
    for (i = 0; i < nvr; i++)
    {
        value[i] = i(vr[i]);
    }
    return fmi2OK;
}

fmi2Status fmi2GetBoolean
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , fmi2Boolean value[])
{
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    size_t i;
    for (i = 0; i < nvr; i++)
    {
        value[i] = b(vr[i]);
    }
    return fmi2OK;
}

fmi2Status fmi2GetString
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , fmi2String value[])
{
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    size_t i;
    for (i = 0; i < nvr; i++)
    {
        value[i] = s(vr[i]);
    }
    return fmi2OK;
}

fmi2Status fmi2SetReal
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , const fmi2Real value[])
{
    size_t i;
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    for (i = 0; i < nvr; i++)
    {
        logf(fmi2OK, "fmi2SetReal vr = %d, value = %lf", vr[i], value[i]);
        r(vr[i],0) = value[i];
    }
    return fmi2OK;
}

fmi2Status fmi2SetInteger
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , const fmi2Integer value[])
{
    size_t i;
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    for (i = 0; i < nvr; i++)
    {
        i(vr[i]) = value[i];
    }
    return fmi2OK;
}

fmi2Status fmi2SetBoolean
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , const fmi2Boolean value[])
{
    size_t i;
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    for (i = 0; i < nvr; i++)
    {
        b(vr[i]) = value[i];
    }
    return fmi2OK;
}

fmi2Status fmi2SetString
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , const fmi2String value[])
{
    size_t i;
    for (i = 0; i < nvr; i++)
    {
        s(vr[i]) = value[i];
    }
    return fmi2OK;
}

fmi2Status fmi2SetRealInputDerivatives
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , const fmi2Integer dvr[]
    , const fmi2Real value[])
{
    size_t i;
    if (component == NULL)
    {
        return fmi2Fatal;
    }
    for (i = 0; i < nvr; i++)
    {
        if (dvr[i] < 1 || dvr[i] > MAX_INPUT_DERIVATIVE_ORDER)
        {
            logf(fmi2Error, "fmi2SetRealInputDerivatives vr = %d, d = %d, value = %lf", vr[i], dvr[i], value[i]);
            return fmi2Error;
        }
        else
        {
            logf(fmi2OK, "fmi2SetRealInputDerivatives vr = %d, d = %d, value = %lf", vr[i], dvr[i], value[i]);
            r(vr[i],dvr[i]) = value[i];
        }
    }
    return fmi2OK;
}

fmi2Status fmi2GetRealOutputDerivatives
    ( fmi2Component component
    , const fmi2ValueReference vr[]
    , size_t nvr
    , const fmi2Integer dvr[]
    , fmi2Real value[])
{
    size_t i;
    for (i = 0; i < nvr; i++)
    {
        return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status fmi2CancelStep(fmi2Component component)
{
    return fmi2Error;
}

fmi2Status fmi2GetStatus
    ( fmi2Component component
    , const fmi2StatusKind status
    , fmi2Status* value)
{
    *value = fmi2OK;
    return fmi2OK;
}

fmi2Status fmi2GetRealStatus
    ( fmi2Component component
    , const fmi2StatusKind status
    , fmi2Real* value)
{
    return fmi2OK;
}

fmi2Status fmi2GetIntegerStatus
    ( fmi2Component component
    , const fmi2StatusKind status
    , fmi2Integer* value)
{
    return fmi2OK;
}

fmi2Status fmi2GetBooleanStatus
    ( fmi2Component component
    , const fmi2StatusKind status
    , fmi2Boolean* value)
{
    return fmi2OK;
}

fmi2Status fmi2GetStringStatus
    ( fmi2Component component
    , const fmi2StatusKind status
    , fmi2String* value)
{
    return fmi2OK;
}

#endif // TEMPLATE_H


/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 * 
 * Single mass force-to-displacement oscillator.
 */
#include <fmi2Functions.h>
#include <cvode/cvode.h>
#include <nvector/nvector_serial.h>
#include <cvode/cvode_dense.h>
#include <cvode/cvode_diag.h>
#include <sundials/sundials_types.h>

#define MAX_INPUT_DERIVATIVE_ORDER 10
#define NUMBER_OF_REALS 6
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2, 3, 4, 5, 1, 2};

#define vr_tauOther 0
#define _tauOther r(vr_tauOther,0)
#define _omegaThis r(1,0)
#define _J r(2,0)
#define _c r(3,0)
#define _d r(4,0)
#define _phiThis0 r(5,0)
#define _omegaThis0 r(6,0)

#define NUMBER_OF_STATES 2
#define _phiThisS NV_Ith_S(y,0)
#define _omegaThisS NV_Ith_S(y,1)
#define _dphiThisS NV_Ith_S(dy,0)
#define _domegaThisS NV_Ith_S(dy,1)
#define Jac(i,j) DENSE_ELEM(J,i,j)

#define _y _internal.y
#define _cvode _internal.cvode
struct Internal
{
    N_Vector y;
    void* cvode;
};

#include <template.h>

static int f(realtype t, N_Vector y, N_Vector dy, void *user_data)
{
    fmi2Component component = user_data;
    fmi2Real FOther = interp(component, vr_tauOther, t - _t);

    _dphiThisS = _omegaThisS;
    _domegaThisS = -_c / _J * _phiThisS - _d / _J * _omegaThisS + 1. / _J * FOther;

    return CV_SUCCESS;
}

static int Jacobian(long int N, realtype t, N_Vector y, N_Vector fy, DlsMat J, void *user_data, N_Vector tmp1, N_Vector tmp2, N_Vector tmp3)
{
    fmi2Component component = user_data;

    Jac(0,0) = 0.;
    Jac(0,1) = 1.;
    Jac(1,0) = -_c / _J;
    Jac(1,1) = -_d / _J;

    return CV_SUCCESS;
}

void InstantiateInternal(fmi2Component component)
{
    _cvode = CVodeCreate(CV_BDF, CV_NEWTON);
    _y = N_VNew_Serial(NUMBER_OF_STATES);
}

void FreeInternal(fmi2Component component)
{
    N_VDestroy_Serial(_y);
    CVodeFree(&_cvode);
}

fmi2Status InitializeIntegrator(fmi2Component component)
{
    realtype reltol = 0.;
    realtype abstol = 1e-8;
    log(fmi2OK, "Hello from InitializeIntegrator!");
    if (CVodeInit(_cvode, f, _t, _y) != CV_SUCCESS)
    {
        return fmi2Error;
    }
    if (CVodeSetUserData(_cvode, component) != CV_SUCCESS)
    {
        return fmi2Error;
    }
    if (CVodeSStolerances(_cvode, reltol, abstol) != CV_SUCCESS)
    {
      return fmi2Error;
    }
    if (CVDense(_cvode, NUMBER_OF_STATES) != CV_SUCCESS)
    {
        return fmi2Error;
    }
    if (CVDlsSetDenseJacFn(_cvode, Jacobian) != CV_SUCCESS)
    {
        return fmi2Error;
    }
    return fmi2OK;
}

void StartInitialization(fmi2Component component)
{
    _tauOther = 0.;
    _J = 1.;
    _c = 1.;
    _d = 1.;
    _phiThis0 = 0.1;
    _omegaThis0 = 0.1;
}

fmi2Status FinishInitialization(fmi2Component component)
{
    N_Vector y = _y;
    _phiThisS = _phiThis0;
    _omegaThisS = _omegaThis0;
	_omegaThis = _omegaThis0;
    return InitializeIntegrator(component);
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    N_Vector y = _y;
    realtype tReached;
    if (CVode(_cvode, _t + h, _y, &tReached, CV_NORMAL) != CV_SUCCESS)
    {
        return fmi2Error;
    }
    _omegaThis = _omegaThisS;
    return fmi2OK;
}

fmi2Status OutputUpdate(fmi2Component component)
{
	return fmi2OK;
}

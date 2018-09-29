/*
 * MIT License
 * 
 * Copyright (c) 2017 Slaven Glumac
 *
 * Two mass oscillator with damping between two masses.
 */
#include <fmi2Functions.h>
#include <cvode/cvode.h>
#include <nvector/nvector_serial.h>
#include <cvode/cvode_dense.h>
#include <sundials/sundials_dense.h>
#include <sundials/sundials_types.h>

#define MAX_INPUT_DERIVATIVE_ORDER 0
#define NUMBER_OF_REALS 17
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};

#define _x_1 r(0,0)
#define _v_1 r(1,0)

#define _x_2 r(2,0)
#define _v_2 r(3,0)

#define _m_1 r(4,0)
#define _c_1 r(5,0)
#define _d_1 r(6,0)

#define _x0_1 r(7,0)
#define _v0_1 r(8,0)

#define _m_2 r(9,0)
#define _c_2 r(10,0)
#define _d_2 r(11,0)

#define _x0_2 r(12,0)
#define _v0_2 r(13,0)

#define _ck r(14,0)
#define _dk r(15,0)

#define _F_1 r(16,0)

#define NUMBER_OF_STATES 4

#define _x1S NV_Ith_S(y,0)
#define _v1S NV_Ith_S(y,1)
#define _x2S NV_Ith_S(y,2)
#define _v2S NV_Ith_S(y,3)

#define _dx1S NV_Ith_S(dy,0)
#define _dv1S NV_Ith_S(dy,1)
#define _dx2S NV_Ith_S(dy,2)
#define _dv2S NV_Ith_S(dy,3)

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

    _dx1S = _v1S;
    _dv1S = -(_c_1 + _ck) / _m_1 * _x1S - (_d_1 + _dk) / _m_1 * _v1S + _ck / _m_1 * _x2S +  _dk / _m_1 * _v2S;
    _dx2S = _v2S;
    _dv2S = _ck / _m_2 * _x1S + _dk / _m_2 * _v1S - (_c_2 + _ck) / _m_2 * _x2S - (_d_2 + _dk) / _m_2 * _v2S;

    return CV_SUCCESS;
}

static int Jacobian(long int N, realtype t, N_Vector y, N_Vector fy, DlsMat J, void *user_data, N_Vector tmp1, N_Vector tmp2, N_Vector tmp3)
{
    fmi2Component component = user_data;

    Jac(0,0) = 0.; Jac(0,1) = 1.; Jac(0,2) = 0.; Jac(0,3) = 0.;
    Jac(1,0) = -(_c_1 + _ck) / _m_1; Jac(1,1) = -(_d_1 + _dk) / _m_1; Jac(1,2) = _ck / _m_1; Jac(1,3) = _dk / _m_1;
    Jac(2,0) = 0.; Jac(2,1) = 0.; Jac(2,2) = 0.; Jac(2,3) = 1.;
    Jac(3,0) = _ck / _m_2; Jac(3,1) = _dk / _m_2; Jac(3,2) = -(_c_2 + _ck) / _m_2; Jac(3,3) = -(_d_2 + _dk) / _m_2;

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
    realtype reltol = 1e-8;
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
    _m_1 = 10.;
    _c_1 = 1.;
    _d_1 = 1.;

    _x0_1 = 0.1;
    _v0_1 = 0.1;

    _m_2 = 10.;
    _c_2 = 1.;
    _d_2 = 2.;

    _x0_2 = 0.2;
    _v0_2 = 0.1;

    _ck = 1.;
    _dk = 1.;

    _x_1 = _x0_1;
    _v_1 = _v0_1;

    _x_2 = _x0_2;
    _v_2 = _v0_2;

}

fmi2Status FinishInitialization(fmi2Component component)
{
    N_Vector y = _y;
    _x1S = _x0_1;
    _v1S = _v0_1;
    _x2S = _x0_2;
    _v2S = _v0_2;
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
    _x_1 = _x1S;
    _v_1 = _v1S;
    _x_2 = _x2S;
    _v_2 = _v2S;
    return fmi2OK;
}

fmi2Status OutputUpdate(fmi2Component component)
{
	_F_1 = _ck * _x_1 + _dk * _v_1 - _ck * _x_2 - _dk * _v_2;
	return fmi2OK;
}

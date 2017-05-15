#include <fmi2Functions.h>
#include <cvode/cvode.h>
#include <nvector/nvector_serial.h>
#include <cvode/cvode_dense.h>
#include <cvode/cvode_diag.h>
#include <sundials/sundials_types.h>

#define MAX_INPUT_DERIVATIVE_ORDER 10
#define NUMBER_OF_REALS 10
#define NUMBER_OF_INTEGERS 0
#define NUMBER_OF_BOOLEANS 0
#define NUMBER_OF_STRINGS 0

const fmi2ValueReference ivrs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 8, 9};

#define vr_xOther 0
#define vr_vOther 1
#define _xOther r(vr_xOther,0)
#define _vOther r(vr_vOther,0)
#define _FThis r(2,0)
#define _m r(3,0)
#define _c r(4,0)
#define _d r(5,0)
#define _ck r(6,0)
#define _dk r(7,0)
#define _x0 r(8,0)
#define _v0 r(9,0)
#define _xThis r(10,0)
#define _vThis r(11,0)

#define NUMBER_OF_STATES 2
#define _xS NV_Ith_S(y,0)
#define _vS NV_Ith_S(y,1)
#define _dxS NV_Ith_S(dy,0)
#define _dvS NV_Ith_S(dy,1)
#define Jac(i,j) DENSE_ELEM(J,i,j)

#define _y _internal.y
#define _cvode _internal.cvode
struct Internal
{
    N_Vector y;
    void* cvode;
};

#include <template.h>

#include <stdio.h>
static int f(realtype t, N_Vector y, N_Vector dy, void *user_data)
{
    fmi2Component component = user_data;
    fmi2Real xOther = interp(component, vr_xOther, t - _t);
    fmi2Real vOther = interp(component, vr_vOther, t - _t);

    _dxS = _vS;
    _dvS = -(_c + _ck) / _m * _xS;
    _dvS -= (_d + _dk) / _m * _vS;
    _dvS += _ck / _m * xOther;
    _dvS += _dk / _m * vOther;

    return CV_SUCCESS;
}

static int Jacobian(long int N, realtype t, N_Vector y, N_Vector fy, DlsMat J, void *user_data, N_Vector tmp1, N_Vector tmp2, N_Vector tmp3)
{
    fmi2Component component = user_data;

    Jac(0,0) = 0.;
    Jac(0,1) = 1.;
    Jac(1,0) = -(_c + _ck) / _m;
    Jac(1,1) = -(_d + _dk) / _m;

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
    _xOther = 0.;
    _vOther = 0.;
    _m = 1.;
    _c = 1.;
    _d = 1.;
    _ck = 1.;
    _dk = 1.;
    _x0 = 0.1;
    _v0 = 0.1;
}

fmi2Status FinishInitialization(fmi2Component component)
{
    N_Vector y = _y;
    _xS = _x0;
    _vS = _v0;
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
    _xThis = _xS;
    _vThis = _vS;
    _FThis = _ck * _xThis + _dk * _vThis - _ck * _xOther - _dk * _vOther;
    return fmi2OK;
}


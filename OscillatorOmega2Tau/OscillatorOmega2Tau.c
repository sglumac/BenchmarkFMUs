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

const fmi2ValueReference ivrs[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 7, 8};

#define vr_omegaOther 0
#define _omegaOther r(vr_omegaOther,0)
#define _tauThis r(1,0)
#define _J r(2,0)
#define _c r(3,0)
#define _d r(4,0)
#define _ck r(5,0)
#define _dk r(6,0)
#define _phiThis0 r(7,0)
#define _omegaThis0 r(8,0)
#define _phiOther0 r(9,0)
#define _phiThis r(7,0)
#define _omegaThis r(8,0)
#define _phiOther r(9,0)

#define NUMBER_OF_STATES 3
#define _phiThisS NV_Ith_S(y,0)
#define _omegaThisS NV_Ith_S(y,1)
#define _phiOtherS NV_Ith_S(y,2)
#define _dphiS NV_Ith_S(dy,0)
#define _domegaThisS NV_Ith_S(dy,1)
#define _dphiOtherS NV_Ith_S(dy,2)
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
    fmi2Real omegaOther = interp(component, vr_omegaOther, t - _t);

    _dphiS = _omegaThisS;
    _domegaThisS = -(_c + _ck) / _J * _phiThisS;
    _domegaThisS -= (_d + _dk) / _J * _omegaThisS;
    _domegaThisS += _ck / _J * _phiOtherS;
    _domegaThisS += _dk / _J * omegaOther;
	_dphiOtherS = omegaOther;

    return CV_SUCCESS;
}

static int Jacobian(long int N, realtype t, N_Vector y, N_Vector fy, DlsMat J, void *user_data, N_Vector tmp1, N_Vector tmp2, N_Vector tmp3)
{
    fmi2Component component = user_data;

    Jac(0,0) = 0.;
    Jac(0,1) = 1.;
	Jac(0,2) = 0.;
    Jac(1,0) = -(_c + _ck) / _J;
    Jac(1,1) = -(_d + _dk) / _J;
	Jac(1,2) = _ck / _J;
	Jac(2,0) = 0.;
    Jac(2,1) = 0.;
	Jac(2,2) = 0.;

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
    realtype reltol = 1e-3;
    realtype abstol = 1e-3;
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
    _phiOther = 0.;
    _omegaOther = 0.;
    _J = 1.;
    _c = 1.;
    _d = 1.;
    _ck = 1.;
    _dk = 1.;
    _phiThis0 = 0.1;
    _omegaThis0 = 0.1;
}

fmi2Status FinishInitialization(fmi2Component component)
{
    N_Vector y = _y;
    _phiThisS = _phiThis0;
    _omegaThisS = _omegaThis0;
	_phiOtherS = _phiOther0;
	_phiOther = _phiOther0;
     OutputUpdate(component);
    return InitializeIntegrator(component);
}

fmi2Status StateUpdate(fmi2Component component, fmi2Real h)
{
    N_Vector y = _y;
    realtype tReached;
    if (CVode(_cvode, _t + h, _y, &tReached, CV_NORMAL) != CV_SUCCESS)
    {
		log(fmi2Error, "The integration failed!");
        return fmi2Error;
    }
    _phiThis = _phiThisS;
    _omegaThis = _omegaThisS;
	_phiOther = _phiOtherS; 
    return fmi2OK;
}

fmi2Status OutputUpdate(fmi2Component component)
{
	_tauThis = _ck * _phiThis + _dk * _omegaThis - _ck * _phiOther - _dk * _omegaOther;
	return fmi2OK;
}

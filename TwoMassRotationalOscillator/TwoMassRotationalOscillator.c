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

#define _phi_O2T r(0,0)
#define _omega_O2T r(1,0)

#define _phi_T2O r(2,0)
#define _omega_T2O r(3,0)

#define _J_O2T r(4,0)
#define _c_O2T r(5,0)
#define _d_O2T r(6,0)

#define _phi0_O2T r(7,0)
#define _omega0_O2T r(8,0)

#define _J_T2O r(9,0)
#define _c_T2O r(10,0)
#define _d_T2O r(11,0)

#define _phi0_T2O r(12,0)
#define _omega0_T2O r(13,0)

#define _ck r(14,0)
#define _dk r(15,0)

#define _tau_O2T r(16,0)

#define NUMBER_OF_STATES 4

#define _phiS_O2T NV_Ith_S(y,0)
#define _omegaS_O2T NV_Ith_S(y,1)
#define _phiS_T2O NV_Ith_S(y,2)
#define _omegaS_T2O NV_Ith_S(y,3)

#define _dphiS_O2T NV_Ith_S(dy,0)
#define _domegaS_O2T NV_Ith_S(dy,1)
#define _dphiS_T2O NV_Ith_S(dy,2)
#define _domegaS_T2O NV_Ith_S(dy,3)

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

    _dphiS_O2T = _omegaS_O2T;
    _domegaS_O2T = -(_c_O2T + _ck) / _J_O2T * _phiS_O2T - (_d_O2T + _dk) / _J_O2T * _omegaS_O2T + _ck / _J_O2T * _phiS_T2O +  _dk / _J_O2T * _omegaS_T2O;
    _dphiS_T2O = _omegaS_T2O;
    _domegaS_T2O = _ck / _J_T2O * _phiS_O2T + _dk / _J_T2O * _omegaS_O2T - (_c_T2O + _ck) / _J_T2O * _phiS_T2O - (_d_T2O + _dk) / _J_T2O * _omegaS_T2O;

    return CV_SUCCESS;
}

static int Jacobian(long int N, realtype t, N_Vector y, N_Vector fy, DlsMat J, void *user_data, N_Vector tmp1, N_Vector tmp2, N_Vector tmp3)
{
    fmi2Component component = user_data;

    Jac(0,0) = 0.; Jac(0,1) = 1.; Jac(0,2) = 0.; Jac(0,3) = 0.;
    Jac(1,0) = -(_c_O2T + _ck) / _J_O2T; Jac(1,1) = -(_d_O2T + _dk) / _J_O2T; Jac(1,2) = _ck / _J_O2T; Jac(1,3) = _dk / _J_O2T;
    Jac(2,0) = 0.; Jac(2,1) = 0.; Jac(2,2) = 0.; Jac(2,3) = 1.;
    Jac(3,0) = _ck / _J_T2O; Jac(3,1) = _dk / _J_T2O; Jac(3,2) = -(_c_T2O + _ck) / _J_T2O; Jac(3,3) = -(_d_T2O + _dk) / _J_T2O;

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
    _J_O2T = 10.;
    _c_O2T = 1.;
    _d_O2T = 1.;

    _phi0_O2T = 0.1;
    _omega0_O2T = 0.1;

    _J_T2O = 10.;
    _c_T2O = 1.;
    _d_T2O = 2.;

    _phi0_T2O = 0.2;
    _omega0_T2O = 0.1;

    _ck = 1.;
    _dk = 1.;

    _phi_O2T = _phi0_O2T;
    _omega_O2T = _omega0_O2T;

    _phi_T2O = _phi0_T2O;
    _omega_T2O = _omega0_T2O;

}

fmi2Status FinishInitialization(fmi2Component component)
{
    N_Vector y = _y;
    _phiS_O2T = _phi0_O2T;
    _phi_O2T = _phi0_O2T;
    _omegaS_O2T = _omega0_O2T;
    _omega_O2T = _omega0_O2T;
    _phiS_T2O = _phi0_T2O;
    _phi_T2O = _phi0_T2O;
    _omegaS_T2O = _omega0_T2O;
    _omega_T2O = _omega0_T2O;
    OutputUpdate(component);
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
    _phi_O2T = _phiS_O2T;
    _omega_O2T = _omegaS_O2T;
    _phi_T2O = _phiS_T2O;
    _omega_T2O = _omegaS_T2O;
    return fmi2OK;
}

fmi2Status OutputUpdate(fmi2Component component)
{
	
	_tau_O2T = _ck * _phi_O2T + _dk * _omega_O2T - _ck * _phi_T2O - _dk * _omega_T2O;
	printf("2M: _ck = %lf, _phi_O2T = %lf, _dk = %lf, _omega_O2T = %lf, _phi_T2O = %lf, _omega_T2O = %lf, tau_O2T = %lf\n", _ck, _phi_O2T, _dk, _omega_O2T, _phi_T2O, _omega_T2O, _tau_O2T); 
	return fmi2OK;
}

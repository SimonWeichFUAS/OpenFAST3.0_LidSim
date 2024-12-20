% This function computes derivatives of continuous states
function [u, xdot, m] = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage)

    u.InputData     = GetInputData(iStep, RK4_stage, u, p);
    
    xdot.qt(p.DOF_TFA1, 1)  = x.qdt(p.DOF_TFA1);
    xdot.qt(p.DOF_GeAz, 1)  = x.qdt(p.DOF_GeAz);

    xdot.qdt(p.DOF_TFA1, 1) = u.InputData(p.DOF_TFA1);
    xdot.qdt(p.DOF_GeAz, 1) = u.InputData(p.DOF_GeAz);

    m.qd2t = xdot.qdt;

end
 
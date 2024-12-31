% This function computes derivatives of continuous states
function [u, xdot, m] = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage)
    
    % Initializtation 
    u                       = GetInputData(iStep, RK4_stage, u, p);
    m                       = SetCoordSy(p, x, m, u);
    m                       = CalculatePositions(p, x, m);
    m                       = CalculateAngularPosVelPAcc(p, x, m);
    m                       = CalculateLinearVelPAcc(p, x, m);
    m                       = CalculateForcesMoments(p, x, m, u);

    [u, m]                  = FillAugMat(p, x, m, u);

    m.AugMat_factor         = m.AugMat( p.DOFs.SrtPS(1:p.DOFs.NActvDOF), p.DOFs.SrtPSNAUG(1:p.DOFs.NActvDOF));
    m.SolnVec               = m.AugMat( p.DOFs.SrtPS(1:p.DOFs.NActvDOF), p.DOFs.SrtPSNAUG(1+p.DOFs.NActvDOF));

    m.SolnVec               = linsolve(m.AugMat_factor, m.SolnVec);
    
    xdot.qt(p.DOF_TFA1, 1)  = x.qdt(p.DOF_TFA1);
    xdot.qt(p.DOF_GeAz, 1)  = x.qdt(p.DOF_GeAz);

    xdot.qdt(p.DOF_TFA1, 1) = m.SolnVec(p.DOF_TFA1);
    xdot.qdt(p.DOF_GeAz, 1) = m.SolnVec(p.DOF_GeAz);

    m.qd2t = xdot.qdt;

end
 
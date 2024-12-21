% This function computes derivatives of continuous states
function [u, xdot, m] = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage)

    u.InputData             = GetInputData(iStep, RK4_stage, u, p);
    
    DataIndex               = 1;
    for iRow = 1:p.DOFs.NActvDOF
        for iCol = 1:p.DOFs.NActvDOF
            m.AugMat_factor(iRow, iCol) = u.InputData(DataIndex);
            DataIndex   = DataIndex + 1;
        end
    end

    for iVecEl = 1:p.DOFs.NActvDOF
        m.SolnVec(iVecEl)   = u.InputData(iVecEl + 4);
    end

    m.SolnVec = linsolve(m.AugMat_factor, m.SolnVec);
    
    xdot.qt(p.DOF_TFA1, 1)  = x.qdt(p.DOF_TFA1);
    xdot.qt(p.DOF_GeAz, 1)  = x.qdt(p.DOF_GeAz);

    xdot.qdt(p.DOF_TFA1, 1) = m.SolnVec(p.DOF_TFA1);
    xdot.qdt(p.DOF_GeAz, 1) = m.SolnVec(p.DOF_GeAz);

    m.qd2t = xdot.qdt;

end
 
% -------------------------------------------------------------------------
%
% This function computes derivatives of continuous states
%
% -------------------------------------------------------------------------
%
% Options for p.SimMode
%       - v0:   This mode replicate the complete functionality of the
%               ElastoDyn module. As primary inputs it uses the aerodynamic
%               loads at the blade nodes.
%
%       - v1:   This mode is the first step of simplifiying the model.
%               Here, the computed loads (by the ElastoDyn) at the hub are
%               being used as the primary inputs. 
%                
%       - v2:   The second simplification concerns the population of the
%               augmented matrix. Here, elements which do not lay on the
%               main diagonal of the mass matrix are not populated.
%               Furhter, the computation of the solution vector does not
%               require a solver for a linear system anymore. An
%               elementwise division leads to the same results.
%
% -------------------------------------------------------------------------
function [u, xdot, m] = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage)
    
    switch p.SimMode
        case 'v0'
            % Initializtation
            u                       = GetInputData(iStep, RK4_stage, u, p);
            m                       = SetCoordSy(p, x, m, u);
            m                       = CalculatePositions(p, m);
            m                       = CalculateAngularPosVelPAcc(p, x, m);
            m                       = CalculateLinearVelPAcc(p, x, m);
            m                       = CalculateForcesMoments(p, m, u);
            
            % Population of the augmented matrix
            [u, m]                  = FillAugMat(p, x, m, u);
            
            % Computaion of the approximated accelerations
            m.AugMat_factor         = m.AugMat( p.DOFs.SrtPS(1:p.DOFs.NActvDOF), p.DOFs.SrtPSNAUG(1:p.DOFs.NActvDOF));
            m.SolnVec               = m.AugMat( p.DOFs.SrtPS(1:p.DOFs.NActvDOF), p.DOFs.SrtPSNAUG(1+p.DOFs.NActvDOF));

            m.SolnVec               = linsolve(m.AugMat_factor, m.SolnVec);

        case 'v1'
            % Initializtation
            u                       = GetInputData_v1(iStep, RK4_stage, u, p);
            m                       = SetCoordSy(p, x, m, u);
            m                       = CalculatePositions(p, m);
            m                       = CalculateAngularPosVelPAcc(p, x, m);
            m                       = CalculateLinearVelPAcc(p, x, m);
            m                       = CalculateForcesMoments_v1(p, m, u);

            % Population of the augmented matrix
            [u, m]                  = FillAugMat(p, x, m, u);

            % Computaion of the approximated accelerations
            m.AugMat_factor         = m.AugMat( p.DOFs.SrtPS(1:p.DOFs.NActvDOF), p.DOFs.SrtPSNAUG(1:p.DOFs.NActvDOF));
            m.SolnVec               = m.AugMat( p.DOFs.SrtPS(1:p.DOFs.NActvDOF), p.DOFs.SrtPSNAUG(1+p.DOFs.NActvDOF));

            m.SolnVec               = linsolve(m.AugMat_factor, m.SolnVec);

        case 'v2'
            % Initializtation
            u                           = GetInputData_v1(iStep, RK4_stage, u, p);
            m                           = SetCoordSy(p, x, m, u);
            m                           = CalculatePositions(p, m);
            m                           = CalculateAngularPosVelPAcc(p, x, m);
            m                           = CalculateLinearVelPAcc(p, x, m);
            m                           = CalculateForcesMoments_v1(p, m, u);

            % Population of the augmented matrix
            [u, m]                      = FillAugMat_v2(p, x, m, u);
            
            % Computaion of the approximated accelerations
            for I = 1:p.DOFs.NActvDOF
                m.AugMat_factor(I, 1)   = m.AugMat( p.DOFs.SrtPS(I), p.DOFs.SrtPSNAUG(I));
            end

            m.SolnVec                   = m.AugMat( p.DOFs.SrtPS(1:p.DOFs.NActvDOF), p.DOFs.SrtPSNAUG(1+p.DOFs.NActvDOF));

            m.SolnVec                   = m.SolnVec ./ m.AugMat_factor;
    end

    xdot.qt(p.DOF_TFA1, 1)  = x.qdt(p.DOF_TFA1);
    xdot.qt(p.DOF_GeAz, 1)  = x.qdt(p.DOF_GeAz);

    xdot.qdt(p.DOF_TFA1, 1) = m.SolnVec(p.DOF_TFA1);
    xdot.qdt(p.DOF_GeAz, 1) = m.SolnVec(p.DOF_GeAz);

    m.qd2t = xdot.qdt;

end
 
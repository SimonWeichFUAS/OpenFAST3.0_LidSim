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
%       - v3:   This mode is not considered as a simplification but its
%               functionality is essential for future simplifications.
%               Here, the only input parameter for the simulation are the
%               three forces and momentes acting on the rotor hub. In order
%               to reduce the number of inputs compared to previous modes,
%               the computation of partial loads has been re-integrated
%               into the framework.
%
%       - v4:   This mode resembles further simplifications of the whole
%               framwork, where the whole set-up process of determining
%               coordinate systems and calculating loads, motions etc. is
%               concentrated into one calulation script and the main
%               function, which populates the augmentes matrix.
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

        case 'v3'
            % Initializtation
            u                           = GetInputData_v2(iStep, RK4_stage, u, p);
            m                           = SetCoordSy(p, x, m, u);
            m                           = CalculatePositions(p, m);
            m                           = CalculateAngularPosVelPAcc(p, x, m);
            m                           = CalculateLinearVelPAcc(p, x, m);
            m                           = CalculateForcesMoments_v2(p, m, u);

            % Population of the augmented matrix
            [u, m]                      = FillAugMat_v2(p, x, m, u);
            
            % Computaion of the approximated accelerations
            for I = 1:p.DOFs.NActvDOF
                m.AugMat_factor(I, 1)   = m.AugMat( p.DOFs.SrtPS(I), p.DOFs.SrtPSNAUG(I));
            end

            m.SolnVec                   = m.AugMat( p.DOFs.SrtPS(1:p.DOFs.NActvDOF), p.DOFs.SrtPSNAUG(1+p.DOFs.NActvDOF));

            m.SolnVec                   = m.SolnVec ./ m.AugMat_factor;
			
		case 'v4'
            % Initializtation
            [FrcPRott,MomLPRott, GenTrq]            = GetInputData_v1(iStep, RK4_stage, u);
%             m                                     = SetCoordSy_v1(m);
%             m                                     = CalculatePositions_v1(m);
%             m                                     = CalculateAngularPosVelPAcc_v1(p, x, m);
%             m                                     = CalculateLinearVelPAcc_v1(p, x, m);
            [FrcONcRtt,MomBNcRtt]                   = CalculateForcesMoments_v4(p, x, FrcPRott,MomLPRott);

            % Population of the augmented matrix
            [M, F]                                  = FillAugMat_v3(p, x, MomLPRott,FrcONcRtt,MomBNcRtt, GenTrq);
            
            % Computaion of the approximated accelerations
            m.SolnVec                               = F./M;

        case 'v5'
            % Initializtation
            u                           = GetInputData(iStep, RK4_stage, u, p);
            m                           = SetCoordSy(p, x, m, u);
            m                           = CalculatePositions(p, m);
            m                           = CalculateAngularPosVelPAcc(p, x, m);
            m                           = CalculateLinearVelPAcc(p, x, m);
            m                           = CalculateAeroHubForcesMoments(p, m, u);
            m                           = CalculateGravHubForcesMoments(p, m);
            m                           = CalculateAcceHubForcesMoments(p, m);
            m                           = CalculateForcesMoments_v3(p, m, u);

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
 
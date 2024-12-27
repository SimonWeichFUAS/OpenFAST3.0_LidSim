% This function is used to populate the AugMat matrix for RtHS
function [u, m] = FillAugMat(p, x, m, u)
    
    %% Initialize the matrix 
    m.AugMat(:, :)  = 0;
%     GBoxTrq         = u.GenTrq * p.GBRatio;
    GBoxTrq             = p.GBoxTrqConst;
    
    %% Blades
    % A loop through blade nodes/elements is not necessary since no 
    % DOFs accosiated with the corresponding elements of this motion are enabled
    
    %% Tower
    % Initialize the portions of the mass matrix on and below the diagonal 
    % associated with purely tower DOFs
    for L = 1:p.DOFs.NPTTE
        for I = L:p.DOFs.NPTTE
            m.AugMat(p.DOFs.PTTE(I), p.DOFs.PTTE(L))  = p.YawBrMas * dot( m.RtHS.PLinVelEO(1, :, p.DOFs.PTTE(I)), ...
                                                                          m.RtHS.PLinVelEO(1, :, p.DOFs.PTTE(L)));
        end % This loop is only required if more than one NPTTE DOF contributes
    end % All active (enabled) tower DOFs that contribute to the QD2T-related linear accelerations of the yaw bearing

    TmpVec1     = -p.YawBrMas*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEOt);
    for I = 1:p.DOFs.NPTTE
        m.AugMat(p.DOFs.PTTE(I), p.NAug)    = dot( m.RtHS.PLinVelEO(1, :, p.DOFs.PTTE(I)), TmpVec1);
    end
    

    for J = 2:p.TwrNodes+1
        for L = 1:p.DOFs.NPTTE
            for I = L:p.DOFs.NPTTE
                m.AugMat(p.DOFs.PTTE(I), p.DOFs.PTTE(L))    = m.AugMat(p.DOFs.PTTE(I), p.DOFs.PTTE(L)) ...
                                                            + p.TElmntMass(J-1)*dot( m.RtHS.PLinVelET(1, :, p.DOFs.PTTE(I), J), ...
                                                                                   m.RtHS.PLinVelET(1, :, p.DOFs.PTTE(L), J)) ...
                                                            - p.DHNodes(J)*dot( m.RtHS.PLinVelET(1, :, p.DOFs.PTTE(I), J), ...
                                                                                [0, 0, 0]) ...      % No impact from PFTHydro
                                                            - p.DHNodes(J)*dot( [0, 0, 0], ...
                                                                                [0, 0, 0]);         % No impact from PMFHydro
            end
        end % L - All active (enabled) tower DOFs that contribute to the QD2T-related linear accelerations of the tower

        TmpVec1     = [0,0,0].*p.DHNodes(J) ...  % No impact from FTHydrot
                    - p.TElmntMass(J-1)*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccETt(:,J)');
        TmpVec3     = [0,0,0].*p.DHNodes(J);                    % No impact from MFHydrot
        for I = 1:p.DOFs.NPTTE
            m.AugMat(p.DOFs.PTTE(I), p.NAug)    = m.AugMat(p.DOFs.PTTE(I), p.NAug) ...
                                                + dot( m.RtHS.PLinVelET(1, :, p.DOFs.PTTE(I), J), TmpVec1) ...
                                                + dot( [0, 0, 0], TmpVec3);
        end
    end % J - Tower nodes/elements
     
    if p.DOF_Flag(p.DOF_TFA1)
        m.AugMat( p.DOF_TFA1, p.NAug )  = m.AugMat( p.DOF_TFA1, p.NAug ) ...
                                        - p.KTFA(1,1)*x.qt(p.DOF_TFA1) ...
                                        - p.CTFA(1,1)*x.qdt(p.DOF_TFA1);    % Note that this part needs to be extended if more tower DOFs are included
    end

    %%
    if p.DOF_Flag(p.DOF_TFA1)
        for I = p.DOFs.Diag(p.DOF_TFA1):p.DOFs.NActvDOF
            m.AugMat(p.DOFs.SrtPS(I), p.DOF_TFA1)   = m.AugMat(p.DOFs.SrtPS(I), p.DOF_TFA1) ...
                                                    - dot(  m.RtHS.PLinVelEO(1, :, p.DOF_TFA1), ...
                                                            u.InputData((1:3) + (I-1)*6)) ...
                                                    - dot(  m.RtHS.PAngVelEB(1, :, p.DOF_TFA1), ...
                                                            u.InputData((4:6) + (I-1)*6));
        end

        m.AugMat(p.DOF_TFA1, p.NAug)    = m.AugMat(p.DOF_TFA1, p.NAug) ...
                                        + dot( m.RtHS.PLinVelEO(1, :, p.DOF_TFA1), u.InputData((13:15)) ) ...
                                        + dot( m.RtHS.PAngVelEB(1, :, p.DOF_TFA1), u.InputData((16:18)) );
    end
    
    TmpVec  = p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.PAngVelEG(1, :, p.DOF_GeAz));
    if p.DOF_Flag(p.DOF_GeAz)
        for I = p.DOFs.Diag(p.DOF_GeAz):p.DOFs.NActvDOF
            m.AugMat(p.DOFs.SrtPS(I), p.DOF_GeAz)   =  -dot( m.RtHS.PAngVelEL(1, :, p.DOF_GeAz), ...
                                                                 u.InputData((19:21)));
        end
        
        m.AugMat(p.DOF_GeAz, p.NAug)    = dot( m.RtHS.PAngVelEL(1, :, p.DOF_GeAz), u.InputData((22:24))) - GBoxTrq;

        m.AugMat(p.DOF_GeAz, p.DOF_GeAz)    = m.AugMat(p.DOF_GeAz, p.DOF_GeAz) ...
                                            + dot( m.RtHS.PAngVelEG(1, :, p.DOF_GeAz), TmpVec);

        m.AugMat(p.DOF_GeAz, p.NAug)        = m.AugMat(p.DOF_GeAz, p.NAug) ...
                                            - dot( m.RtHS.AngAccEGt, TmpVec);
    end

    %% Mirror the mass matrix
    for L = 2:p.DOFs.NActvDOF
        for I = 1:L-1
            m.AugMat(p.DOFs.SrtPS(I), p.DOFs.SrtPS(L))  = m.AugMat(p.DOFs.SrtPS(L), p.DOFs.SrtPS(I));
        end
    end

    %% Gearbox friction
%     GBoxEffFac2     = (1/m.RtHS.GBoxEffFac - 1);
% 
%     for I = 1:p.DOFs.NActvDOF
%         m.AugMat(p.DOF_GeAz, p.DOFs.SrtPS(I))   = m.AugMat(p.DOF_GeAz, p.DOFs.SrtPS(I)) ...
%                                                 + GBoxEffFac2*dot(m.RtHS.PAngVelEG(p.DOFs.SrtPS(I), 0, :), TmpVec);
%     end
% 
%     m.AugMat(p.DOF_GeAz, p.NAug)    = m.AugMat(p.DOF_GeAz, p.NAug) ...
%                                     - GBoxEffFac2*(dot(m.RtHSdat.AngAccEGt, TmpVec) + GBoxTrq);
    
end

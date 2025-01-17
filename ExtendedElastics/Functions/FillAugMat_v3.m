% -------------------------------------------------------------------------
%
% This function is used to populate the AugMat matrix for RtHS
%
% -------------------------------------------------------------------------
function [M, F] = FillAugMat_v3(p, x, m, u)
    
    %% Initialize the matrix ;
    M = NaN(2,1);
    F = NaN(2,1);
    GBoxTrq         = u.GenTrq * p.GBRatio;
    

    
    %% Tower M
    M(1)  = p.YawBrMas * dot( m.RtHS.PLinVelEO(1, :, 1), m.RtHS.PLinVelEO(1, :, 1));
    
    for J = 2:p.TwrNodes+1

                M(1)    = M(1) ...
                                                            + p.TElmntMass(J-1)*dot( m.RtHS.PLinVelET(1, :, 1, J), ...
                                                                                   m.RtHS.PLinVelET(1, :, 1, J)) ...
                                                            - p.DHNodes(J)*dot( m.RtHS.PLinVelET(1, :, 1, J), ...
                                                                                [0, 0, 0]) ...      % No impact from PFTHydro
                                                            - p.DHNodes(J)*dot( [0, 0, 0], ...
                                                                                [0, 0, 0]);         % No impact from PMFHydro
    end                
    M(1)   = M(1) ...
                                            - dot(  m.RtHS.PLinVelEO(1, :, 1), ...
                                                    m.RtHS.PFrcONcRt(:, 1)) ...
                                            - dot(  m.RtHS.PAngVelEB(1, :, 1), ...
                                                    m.RtHS.PMomBNcRt(:, 1));

    %% Tower F
    TmpVec1     = -p.YawBrMas*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEOt);

    F(1)    = dot( m.RtHS.PLinVelEO(1, :, 1), TmpVec1);

    for J = 2:p.TwrNodes+1

        TmpVec1     = [0,0,0].*p.DHNodes(J) ...  	            % No impact from FTHydrot
                    - p.TElmntMass(J-1)*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccETt(:,J)');
        TmpVec3     = [0,0,0].*p.DHNodes(J);                    % No impact from MFHydrot

            F(1)    = F(1) ...
                                                + dot( m.RtHS.PLinVelET(1, :, 1, J), TmpVec1) ...
                                                + dot( [0, 0, 0], TmpVec3);
    end % J - Tower nodes/elements
     

    F(1)  = F(1) ...
                                    - p.KTFA(1,1)*x.qt(1) ...
                                    - p.CTFA(1,1)*x.qdt(1);    % Note that this part needs to be extended if more tower DOFs are included

    F(1)    = F(1) ...
                                    + dot( m.RtHS.PLinVelEO(1, :, 1), m.RtHS.FrcONcRtt ) ...
                                    + dot( m.RtHS.PAngVelEB(1, :, 1), m.RtHS.MomBNcRtt );

    %% Rotor original
%     TmpVec  = p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.PAngVelEG(1, :, 2));
%     M(2)    =  -dot( m.RtHS.PAngVelEL(1, :, 2),m.RtHS.PMomLPRot(:, 2))...
%                                         + dot( m.RtHS.PAngVelEG(1, :, 2), TmpVec);
%     F(2)    =  dot( (m.RtHS.PAngVelEL(1, :, 2))', m.RtHS.MomLPRott) ...
%                 - GBoxTrq ...
%                 - dot( m.RtHS.AngAccEGt, TmpVec);

    % check 
%     global F21 F22 F23 M21 M22
%     F21     = [F21,dot( (m.RtHS.PAngVelEL(1, :, 2))', m.RtHS.MomLPRott)];
%     F22     = [F22,- GBoxTrq];
%     F23     = [F23,- dot( m.RtHS.AngAccEGt, TmpVec)];
%     M21     = [M21,- dot( m.RtHS.PAngVelEL(1, :, 2),m.RtHS.PMomLPRot(:, 2))];
%     M22     = [M22, dot( m.RtHS.PAngVelEG(1, :, 2), TmpVec)];


    %% Rotor simplified    
    M(2)    =  3.5264e+08; % mean(M22+M21), since it does not change much (should be rotor inertia J of SLOW)
    F(2)    =  dot( (m.RtHS.PAngVelEL(1, :, 2))', m.RtHS.MomLPRott) ... % (should be M_a of SLOW)
                - GBoxTrq; % (should be M_g*r_GB
    % F23 is 0

end

% -------------------------------------------------------------------------
%
% This function is used to populate the AugMat matrix for RtHS
%
% -------------------------------------------------------------------------
%
% Inputs - Misc.:       m.RtHS.PAngVelEB    -
%                       m.RtHS.PAngVelEL    -     
%                       m.RtHS.PLinVelEO    -
%                       m.RtHS.MomLPRott    -
%                       m.RtHS.FrcONcRtt    -
%                       m.RtHS.MomBNcRtt    -
%
% -------------------------------------------------------------------------
%
% Outputs:              M                   - Mass matrix
%                       F                   - Forcing vector
%
% -------------------------------------------------------------------------
function [M, F] = FillAugMat_v3(p, x, m, u)
    
    %% Initialize the matrix ;
    M               = NaN(2,1);
    F               = NaN(2,1);
    GBoxTrq         = u.GenTrq * p.GBRatio;

    %% Tower original
%     M(1)        = p.YawBrMas * dot( m.RtHS.PLinVelEO(1, :, 1), m.RtHS.PLinVelEO(1, :, 1));
%     
%     for J = 2:p.TwrNodes+1
%         M(1)    = M(1)+ p.TElmntMass(J-1)*dot( m.RtHS.PLinVelET(1, :, 1, J), m.RtHS.PLinVelET(1, :, 1, J));
%     end     % J - Tower nodes/elements
% 
%     M(1)        = M(1)  - dot(  m.RtHS.PLinVelEO(1, :, 1), ...
%                                 m.RtHS.PFrcONcRt(:, 1)) ...
%                         - dot(  m.RtHS.PAngVelEB(1, :, 1), ...
%                                 m.RtHS.PMomBNcRt(:, 1));
% 
%     TmpVec1     = -p.YawBrMas*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEOt);
% 
%     F(1)        = dot( m.RtHS.PLinVelEO(1, :, 1), TmpVec1);
% 
%     for J = 2:p.TwrNodes+1
%         TmpVec2 = - p.TElmntMass(J-1)*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccETt(:,J)');
% 
%         F(1)    = F(1) + dot( m.RtHS.PLinVelET(1, :, 1, J), TmpVec2);
%     end     % J - Tower nodes/elements
%      
%     F(1)        = F(1) - p.KTFA(1,1)*x.qt(1) - p.CTFA(1,1)*x.qdt(1);
% 
%     F(1)        = F(1)  + dot( m.RtHS.PLinVelEO(1, :, 1), m.RtHS.FrcONcRtt ) ...
%                         + dot( m.RtHS.PAngVelEB(1, :, 1), m.RtHS.MomBNcRtt );
    
    % check
%     TmpVec1     = -p.YawBrMas*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEOt);
%     TmpVal      = 0;
%     for J = 2:p.TwrNodes+1
%         TmpVal  = TmpVal + p.TElmntMass(J-1)*dot( m.RtHS.PLinVelET(1, :, 1, J), m.RtHS.PLinVelET(1, :, 1, J));      
%     end        
%     
%     TmpVal2     = 0;
%     for J = 2:p.TwrNodes+1
% 
%         TmpVec2     = - p.TElmntMass(J-1)*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccETt(:,J)');
% 
%         TmpVal2     = TmpVal2 + dot( m.RtHS.PLinVelET(1, :, 1, J), TmpVec2);
%     end % J - Tower nodes/elements
%     
%     global F11 F12 F13 F14 F15 M11 M12 M13 M14
%     F11         = [F11, dot( m.RtHS.PLinVelEO(1, :, 1), TmpVec1)];                                  % Needs to be included:     8.5201e+02 N  (but is small compared to other components!)
%     F12         = [F12, TmpVal];                                                                    % Does not change much:     4.1930e+03 N 
%     F13         = [F13, - p.KTFA(1,1)*x.qt(1) - p.CTFA(1,1)*x.qdt(1)];                              % Needs to be included
%     F14         = [F14, dot( m.RtHS.PLinVelEO(1, :, 1), m.RtHS.FrcONcRtt )];                        % Needs to be included
%     F15         = [F15, dot( m.RtHS.PAngVelEB(1, :, 1), m.RtHS.MomBNcRtt )];                        % Needs to be included
%     M11         = [M11, p.YawBrMas * dot( m.RtHS.PLinVelEO(1, :, 1), m.RtHS.PLinVelEO(1, :, 1))];   % Does not change much:     2.8280e+04 kg
%     M12         = [M12, TmpVal2];                                                                   % Does not change much:     1.0745e+05 kg (~12.5% of the total tower mass)
%     M13         = [M13, - dot(  m.RtHS.PLinVelEO(1, :, 1), m.RtHS.PFrcONcRt(:, 1))];                % Changes at e-3:           9.8663e+05 kg      
%     M14         = [M14, - dot(  m.RtHS.PAngVelEB(1, :, 1), m.RtHS.PMomBNcRt(:, 1))];                % Changes at e-3:           1.2282e+05 kg 

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

    %% Tower simplified
    M(1)    =   1.2452e+06;                                                 % M11+M12+M13+M14, steady state values
    F(1)    =   5.0450e+03 ...                                              % F11+F12                     
                - p.KTFA(1,1)*x.qt(1) - p.CTFA(1,1)*x.qdt(1) ...            % F13
                + dot( [1,0,0] - ( p.AxRedTFA(1,1,p.TTopNode)*x.qt(p.DOF_TFA1) ).*[0,1,0], m.RtHS.FrcONcRtt ) ...    % F14
                + dot( [0, 0, -1.4857e-02], m.RtHS.MomBNcRtt );       % F15

    %% Rotor simplified    
    M(2)    =  3.5264e+08; % mean(M22+M21), since it does not change much (should be rotor inertia J of SLOW)
    F(2)    =  dot( [9.9406e-01; -1.0880e-01; 0], m.RtHS.MomLPRott) ... % (should be M_a of SLOW)
                - GBoxTrq; % (should be M_g*r_GB
    % F23 is 0

end

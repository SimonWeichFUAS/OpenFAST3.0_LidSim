% -------------------------------------------------------------------------
%
% This function is used to calculate the forces and moments resulting from
% accelerational effects acting at the rotor hub
%
% -------------------------------------------------------------------------
function m = CalculateAcceHubForcesMoments(p, m, u)

    %% Forces/Moments at the blade root (S(0)) due to the blade
    for K = 1:p.NumBl
        % Consideration of the tip brake (+TipDrag) can be neglected at this point
        TmpVec1                         = [0; 0; 0];
        m.RtHS.FrcS0BtAcce(:, K)        = TmpVec1;
        m.RtHS.MomH0BtAcce(:, K)        = [0; 0; 0];        % cross(rS0S,TmpVec1)

        for J = 1:p.BldNodes
            TmpVec1                     = - p.BElmntMass(J,K).*( m.RtHS.LinAccESt(:,J,K) );
            TmpVec2                     = cross( m.RtHS.rS0S(:,J,K), TmpVec1 );

            m.RtHS.FrcS0BtAcce(:, K)    = m.RtHS.FrcS0BtAcce(:, K) + TmpVec1;
            m.RtHS.MomH0BtAcce(:, K)    = m.RtHS.MomH0BtAcce(:, K) + TmpVec2;
        end     % J - Number of blade nodes/elements
    end     % K - Number of blades

    %% Forces/Moments at the teeter pin (P) due to the rotor
    TmpVec1                             = -p.HubMass*( m.RtHS.LinAccECt );
    TmpVec2                             = (cross( m.RtHS.rPC, TmpVec1 ))';
    TmpVec                              = p.Hubg1Iner*m.CoordSys.g1*dot( m.CoordSys.g1, m.RtHS.AngVelEH ) ...
                                        + p.Hubg2Iner*m.CoordSys.g2*dot( m.CoordSys.g2, m.RtHS.AngVelEH );
    TmpVec3                             = (cross( -m.RtHS.AngVelEH, TmpVec ))';                                 
    
    % BeamDyn loads (HubPtLoads) are excluded at this point
    m.RtHS.FrcPRottAcce                 = TmpVec1;  
    m.RtHS.MomLPRottAcce                = TmpVec2 + TmpVec3 - (p.Hubg1Iner*m.CoordSys.g1*dot( m.CoordSys.g1, m.RtHS.AngAccEHt ))' ...
                                                            - (p.Hubg2Iner*m.CoordSys.g2*dot( m.CoordSys.g2, m.RtHS.AngAccEHt ))';

    for K = 1:p.NumBl
        m.RtHS.FrcPRottAcce             = m.RtHS.FrcPRottAcce + m.RtHS.FrcS0BtAcce(:,K);
        m.RtHS.MomLPRottAcce            = m.RtHS.MomLPRottAcce + m.RtHS.MomH0BtAcce(:,K);
        
    end     % K - Number of blades

end

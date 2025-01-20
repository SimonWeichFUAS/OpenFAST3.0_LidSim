% -------------------------------------------------------------------------
%
% This function is used to calculate the forces and moments resulting from
% gravitational effects acting at the rotor hub
%
% -------------------------------------------------------------------------
function m = CalculateGravHubForcesMoments(p, m, u)

    %% Forces/Moments at the blade root (S(0)) due to the blade
    for K = 1:p.NumBl
        % Consideration of the tip brake (+TipDrag) can be neglected at this point
        TmpVec1                         = [0; 0; 0];
        m.RtHS.FrcS0BtGrav(:, K)        = TmpVec1;
        m.RtHS.MomH0BtGrav(:, K)        = [0; 0; 0];        % cross(rS0S,TmpVec1)

        for J = 1:p.BldNodes
            TmpVec1                     = - p.BElmntMass(J,K).*( (p.Gravity*m.CoordSys.z2)' );
            TmpVec2                     = cross( m.RtHS.rS0S(:,J,K), TmpVec1 );

            m.RtHS.FrcS0BtGrav(:, K)    = m.RtHS.FrcS0BtGrav(:, K) + TmpVec1;
            m.RtHS.MomH0BtGrav(:, K)    = m.RtHS.MomH0BtGrav(:, K) + TmpVec2;
        end     % J - Number of blade nodes/elements
    end     % K - Number of blades
        
    %% Forces/Moments at the teeter pin (P) due to the rotor
    TmpVec1                             = -p.HubMass*( (p.Gravity*m.CoordSys.z2)' );
    TmpVec2                             = (cross( m.RtHS.rPC, TmpVec1 ))';                               
    
    % BeamDyn loads (HubPtLoads) are excluded at this point
    m.RtHS.FrcPRottGrav                 = TmpVec1;  
    m.RtHS.MomLPRottGrav                = TmpVec2;

    for K = 1:p.NumBl
        m.RtHS.FrcPRottGrav             = m.RtHS.FrcPRottGrav + m.RtHS.FrcS0BtGrav(:,K);
        m.RtHS.MomLPRottGrav            = m.RtHS.MomLPRottGrav + m.RtHS.MomH0BtGrav(:,K);
        
    end     % K - Number of blades

end

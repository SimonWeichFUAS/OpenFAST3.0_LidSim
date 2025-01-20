% -------------------------------------------------------------------------
%
% This function is used to calculate the forces and moments resulting from
% aerodynamic effects acting at the rotor hub
%
% -------------------------------------------------------------------------
function m = CalculateAeroHubForcesMoments(p, m, u)
    
    %% Aerodynamic force/moment force per unit span acting on a blade at point S
    for K = 1:p.NumBl
        for J = 1:p.BldNodes
            NodeNum                 = J;

            m.RtHS.FSAero(1, J, K)  =  u.BladePtLoads(K).Force(1, NodeNum) / p.DRNodes(J);
            m.RtHS.FSAero(2, J, K)  =  u.BladePtLoads(K).Force(3, NodeNum) / p.DRNodes(J);
            m.RtHS.FSAero(3, J, K)  = -u.BladePtLoads(K).Force(2, NodeNum) / p.DRNodes(J);

            m.RtHS.MMAero(1, J, K)  =  u.BladePtLoads(K).Moment(1, NodeNum) / p.DRNodes(J);
            m.RtHS.MMAero(2, J, K)  =  u.BladePtLoads(K).Moment(3, NodeNum) / p.DRNodes(J);
            m.RtHS.MMAero(3, J, K)  = -u.BladePtLoads(K).Moment(2, NodeNum) / p.DRNodes(J);

        end     % J - Number of blades nodes/elements
    end     % K - Number of blades

    %% Forces/Moments at the blade root (S(0)) due to the blade
    for K = 1:p.NumBl
        % Consideration of the tip brake (+TipDrag) can be neglected at this point
        TmpVec1                         = [0; 0; 0];
        m.RtHS.FrcS0BtAero(:, K)        = TmpVec1;
        m.RtHS.MomH0BtAero(:, K)        = [0; 0; 0];        % cross(rS0S,TmpVec1)

        for J = 1:p.BldNodes
            TmpVec1                     = m.RtHS.FSAero(:,J,K)*p.DRNodes(J);
            TmpVec2                     = cross( m.RtHS.rS0S(:,J,K), TmpVec1 );
            TmpVec3                     = m.RtHS.MMAero(:,J,K)*p.DRNodes(J);

            m.RtHS.FrcS0BtAero(:, K)    = m.RtHS.FrcS0BtAero(:, K) + TmpVec1;
            m.RtHS.MomH0BtAero(:, K)    = m.RtHS.MomH0BtAero(:, K) + TmpVec2 + TmpVec3;
        end     % J - Number of blade nodes/elements
    end     % K - Number of blades

    %% Forces/Moments at the teeter pin (P) due to the rotor       
    m.RtHS.FrcPRottAero                 = [0; 0; 0];  
    m.RtHS.MomLPRottAero                = [0; 0; 0];

    for K = 1:p.NumBl
        m.RtHS.FrcPRottAero         = m.RtHS.FrcPRottAero + m.RtHS.FrcS0BtAero(:,K);
        m.RtHS.MomLPRottAero        = m.RtHS.MomLPRottAero + m.RtHS.MomH0BtAero(:,K);
    end     % K - Number of blades

end

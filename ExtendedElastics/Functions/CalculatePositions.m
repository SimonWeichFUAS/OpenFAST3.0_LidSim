function m = CalculatePositions(p, x, m)
    
    %% Position vectors between various points on the wind turbine
    m.RtHS.rVP          = p.rVPxn*m.CoordSys.rf1 + p.rVPzn*m.CoordSys.rf2 - p.rVPyn*m.CoordSys.rf3 + p.OverHang*m.CoordSys.c1;
    
    %% Position vectors of various points on the blades
    for K = 1:p.NumBl

        % Position vector of the tip
        m.RtHS.rS0S(:, p.TipNode, K)    = p.BldFlexL*m.CoordSys.j3(K, :);

        m.RtHS.rQS(:, p.TipNode+1, K)     = m.RtHS.rS0S(:, p.TipNode, K) + p.HubRad*m.CoordSys.j3(K, :)';

        % Position vector of the root
        m.RtHS.rQS(:, 1, K)             = p.HubRad*m.CoordSys.j3(K, :);

        % Positio vector of every node
        for J = 1:p.BldNodes
            m.RtHS.rS0S(:, J, K)        = p.RNodes(J)*m.CoordSys.j3(K, :);

            m.RtHS.rQS(:, J+1, K)         = m.RtHS.rS0S(:, J, K) + p.HubRad*m.CoordSys.j3(K, :)';
        end     % J - Number of blade nodes
    end     % K - Number of blades
end

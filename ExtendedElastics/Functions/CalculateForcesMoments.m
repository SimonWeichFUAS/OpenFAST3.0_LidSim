function m = CalculateForcesMoments(p, x, m)
    
    %% Forces/Moments at the blade root (S(0)) due to the blade
    for K = 1:p.NumBl
        m.RtHS.PFrcS0B(:,K,:)   = 0.0;

        % Influence of the tip brake is neglected
        for J = 1:p.BldNodes
            for I = 1:p.DOFs.NPSE(K)
                TmpVec1                                 = -p.BElmntMass(J,K)*m.RtHS.PLinVelES(1, :, p.DOFs.PSE(K,I), J+1, K);

                m.RtHS.PFrcS0B(:, K, p.DOFs.PSE(K,I))   = m.RtHS.PFrcS0B(:, K, p.DOFs.PSE(K,I)) + TmpVec1';
            end     % I - Active DOFs that contribute to the linear accelerations of blade K
        end     % J - Number of blade nodes/elements
    end     % K - Number of blades

    %% Forces/Moments at the teeter pin (P) due to the rotor
    m.RtHS.PFrcPRot(:,:)    = 0.0;
    for I = 1:p.DOFs.NPCE
        TmpVec1                             = -p.HubMass*m.RtHS.PLinVelEC(1, :, p.DOFs.PCE(I));

        m.RtHS.PFrcPRot(:, p.DOFs.PCE(I))   = TmpVec1;
    end     % I - Active DOFs that contribute to the linear acceleration of the hub CoM

    for K = 1:p.NumBl
        for I = 1:p.DOFs.NPSE(K)

            m.RtHS.PFrcPRot(:, p.DOFs.PSE(K, I))    = m.RtHS.PFrcPRot(:, p.DOFs.PSE(K, I)) + m.RtHS.PFrcS0B(:, K, p.DOFs.PSE(K, I));
        end     % I - Active DOFs that contribute to the linear acceleration of the hub CoM
    end     % K - Number of blades

end

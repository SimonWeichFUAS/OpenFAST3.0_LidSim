% -------------------------------------------------------------------------
%
% This function is used to calculate the forces and moments 
% stored in other states
%
% -------------------------------------------------------------------------
function m = CalculateForcesMoments_v3(p, m, u)

    %% (Partial) Forces/Moments at the blade root (S(0)) due to the blade
    for K = 1:p.NumBl
        m.RtHS.PFrcS0B(:,K,:)   = 0.0;
        m.RtHS.PMomH0B(:,K,:)   = 0.0;

        % Influence of the tip brake is neglected
        for J = 1:p.BldNodes
            for I = 1:p.DOFs.NPSE(K)
                TmpVec1                                 = -p.BElmntMass(J,K)*m.RtHS.PLinVelES(1, :, p.DOFs.PSE(K,I), J+1, K);

                m.RtHS.PFrcS0B(:, K, p.DOFs.PSE(K,I))   = m.RtHS.PFrcS0B(:, K, p.DOFs.PSE(K,I)) ...
                                                            + TmpVec1';
                m.RtHS.PMomH0B(:, K, p.DOFs.PSE(K,I))   = m.RtHS.PMomH0B(:, K, p.DOFs.PSE(K,I)) ...
                                                            + cross( m.RtHS.rS0S(:,J,K), TmpVec1' ); 
            end     % I - Active DOFs that contribute to the linear accelerations of blade K
        end     % J - Number of blade nodes/elements
    end     % K - Number of blades
    
    %% (Partial) Forces/Moments at the teeter pin (P) due to the rotor
    m.RtHS.PFrcPRot(:,:)    = 0.0;
    m.RtHS.PMomLPRot(:,:)   = 0.0;
    for I = 1:p.DOFs.NPCE
        TmpVec1                             = -p.HubMass*m.RtHS.PLinVelEC(1, :, p.DOFs.PCE(I));
        TmpVec2                             = cross( m.RtHS.rPC, TmpVec1 );

        m.RtHS.PFrcPRot(:, p.DOFs.PCE(I))   = TmpVec1;
        m.RtHS.PMomLPRot(:, p.DOFs.PCE(I))  = TmpVec2   - p.Hubg1Iner*m.CoordSys.g1*dot( m.CoordSys.g1, m.RtHS.PAngVelEH(1, :, p.DOFs.PCE(I)) ) ...
                                                        - p.Hubg2Iner*m.CoordSys.g2*dot( m.CoordSys.g2, m.RtHS.PAngVelEH(1, :, p.DOFs.PCE(I)) );
    end     % I - Active DOFs that contribute to the linear acceleration of the hub CoM

    for K = 1:p.NumBl
        for I = 1:p.DOFs.NPSE(K)
            TmpVec                                  = cross( m.RtHS.rPS0(:,K), m.RtHS.PFrcS0B(:,K,p.DOFs.PSE(K,I)) );

            m.RtHS.PFrcPRot(:, p.DOFs.PSE(K, I))    = m.RtHS.PFrcPRot(:, p.DOFs.PSE(K, I)) + m.RtHS.PFrcS0B(:, K, p.DOFs.PSE(K, I));
            m.RtHS.PMomLPRot(:, p.DOFs.PSE(K, I))   = m.RtHS.PMomLPRot(:, p.DOFs.PSE(K, I)) + m.RtHS.PMomH0B(:, K, p.DOFs.PSE(K, I)) + TmpVec;
        end     % I - Active DOFs that contribute to the linear acceleration of the hub CoM
    end     % K - Number of blades

    %% Forces/Moments at the teeter pin (P) due to the rotor

    m.RtHS.FrcPRott                 = m.RtHS.FrcPRottAero   + m.RtHS.FrcPRottGrav   + m.RtHS.FrcPRottAcce;

    m.RtHS.MomLPRott                = m.RtHS.MomLPRottAero  + m.RtHS.MomLPRottGrav  + m.RtHS.MomLPRottAcce;
    
    for I = 1:3
        m.RtHS.MomLPRott            = m.RtHS.MomLPRott      + cross(m.RtHS.rPS0(:,I),m.RtHS.FrcS0BtAero(:, I)) ...
                                                            + cross(m.RtHS.rPS0(:,I),m.RtHS.FrcS0BtGrav(:, I)) ...
                                                            + cross(m.RtHS.rPS0(:,I),m.RtHS.FrcS0BtAcce(:, I));
    end
        
    %% (Partial) Froces/Moments at the nacelle (N) due to the furling structure
    m.RtHS.PFrcVGnRt            = m.RtHS.PFrcPRot;
    m.RtHS.PMomNGnRt            = m.RtHS.PMomLPRot;  
    for I = 1:p.DOFs.NActvDOF
        TmpVec                                  = cross( m.RtHS.rVP, m.RtHS.PFrcPRot(:, p.DOFs.SrtPS(I)) );
        m.RtHS.PMomNGnRt(:, p.DOFs.SrtPS(I))    = m.RtHS.PMomNGnRt(:, p.DOFs.SrtPS(I)) + TmpVec';
    end     % I - Active DOFs
    if p.DOF_Flag(p.DOF_GeAz)
        m.RtHS.PMomNGnRt(:, p.DOF_GeAz)         = m.RtHS.PMomNGnRt(:, p.DOF_GeAz) ...
                                                    - (p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.PAngVelEG(1, :, p.DOF_GeAz) ))';
    end
    
    %% Forces/Moments at at the nacelle (N) due to the furling structure
    % Influences of the furling structure are neglected
    TmpVec3                     = cross( m.RtHS.rVP, m.RtHS.FrcPRott );
    TmpVec                      = p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.AngVelEG );
    TmpVec5                     = cross ( -m.RtHS.AngVelEG, TmpVec );

    m.RtHS.FrcVGnRtt            = m.RtHS.FrcPRott;
    m.RtHS.MomNGnRtt            = m.RtHS.MomLPRott + TmpVec3' + TmpVec5' - (p.GenIner.*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.AngAccEGt ))';

    %% (Partial) Froces/Moments at the base plate (B) / yaw bearing (O) due to nacelle, generator, and rotor
    m.RtHS.PFrcONcRt            = m.RtHS.PFrcVGnRt;
    m.RtHS.PMomBNcRt            = m.RtHS.PMomNGnRt;           % Reasonable, since moment due to the tail can be neglected
    for I = 1:p.DOFs.NPUE
        TmpVec1                             = -p.NacMass*m.RtHS.PLinVelEU(1, :, p.DOFs.PUE(I));
        TmpVec2                             = cross( m.RtHS.rOU, TmpVec1 );
        
        m.RtHS.PFrcONcRt(:, p.DOFs.PUE(I))  = m.RtHS.PFrcONcRt(:, p.DOFs.PUE(I)) + TmpVec1';
        m.RtHS.PMomBNcRt(:, p.DOFs.PUE(I))  = m.RtHS.PMomBNcRt(:, p.DOFs.PUE(I)) ...
                                                + (TmpVec2 - p.Nacd2Iner*m.CoordSys.d2*dot( m.CoordSys.d2, m.RtHS.PAngVelEN(1, :, p.DOFs.PUE(I)) ))';
    end     % I - Active DOFs that contribute to the linear accelerations of the nacelle CoM

    %% Forces/Moments at the yaw bearing (O)
    TmpVec1                     = (-p.NacMass*( p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEUt ))';
    TmpVec2                     = (cross( m.RtHS.rOU, TmpVec1 ))';
    m.RtHS.FrcONcRtt            = m.RtHS.FrcVGnRtt + TmpVec1;
    m.RtHS.MomBNcRtt            = m.RtHS.MomNGnRtt + TmpVec2 ...
                                    - (p.Nacd2Iner*m.CoordSys.d2*dot( m.CoordSys.d2, m.RtHS.AngAccENt ))';

end

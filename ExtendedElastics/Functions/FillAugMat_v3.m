% -------------------------------------------------------------------------
%
% This function is used to populate the AugMat matrix for RtHS
%
% -------------------------------------------------------------------------
function [M, F] = FillAugMat_v3(p, x, m, u)
    
    %% Initialize the matrix 
    m.AugMat(:, :)  = 0;
    GBoxTrq         = u.GenTrq * p.GBRatio;
    
    %% Blades
    % A loop through blade nodes/elements is not necessary since no 
    % DOFs accosiated with the corresponding elements of this motion are enabled
    
    %% Tower
    % Initialize the portions of the mass matrix on and below the diagonal 
    % associated with purely tower DOFs

    m.AugMat(1, 1)  = p.YawBrMas * dot( m.RtHS.PLinVelEO(1, :, 1), ...
                                                                          m.RtHS.PLinVelEO(1, :, 1));

    TmpVec1     = -p.YawBrMas*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEOt);

    m.AugMat(1, 3)    = dot( m.RtHS.PLinVelEO(1, :, 1), TmpVec1);

    
    for J = 2:p.TwrNodes+1

                m.AugMat(1, 1)    = m.AugMat(1, 1) ...
                                                            + p.TElmntMass(J-1)*dot( m.RtHS.PLinVelET(1, :, 1, J), ...
                                                                                   m.RtHS.PLinVelET(1, :, 1, J)) ...
                                                            - p.DHNodes(J)*dot( m.RtHS.PLinVelET(1, :, 1, J), ...
                                                                                [0, 0, 0]) ...      % No impact from PFTHydro
                                                            - p.DHNodes(J)*dot( [0, 0, 0], ...
                                                                                [0, 0, 0]);         % No impact from PMFHydro

        TmpVec1     = [0,0,0].*p.DHNodes(J) ...  	            % No impact from FTHydrot
                    - p.TElmntMass(J-1)*(p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccETt(:,J)');
        TmpVec3     = [0,0,0].*p.DHNodes(J);                    % No impact from MFHydrot

            m.AugMat(1, 3)    = m.AugMat(1, 3) ...
                                                + dot( m.RtHS.PLinVelET(1, :, 1, J), TmpVec1) ...
                                                + dot( [0, 0, 0], TmpVec3);
    end % J - Tower nodes/elements
     

        m.AugMat( 1, 3 )  = m.AugMat( 1, 3 ) ...
                                        - p.KTFA(1,1)*x.qt(1) ...
                                        - p.CTFA(1,1)*x.qdt(1);    % Note that this part needs to be extended if more tower DOFs are included





            m.AugMat(1, 1)   = m.AugMat(1, 1) ...
                                                    - dot(  m.RtHS.PLinVelEO(1, :, 1), ...
                                                            m.RtHS.PFrcONcRt(:, 1)) ...
                                                    - dot(  m.RtHS.PAngVelEB(1, :, 1), ...
                                                            m.RtHS.PMomBNcRt(:, 1));


        m.AugMat(1, 3)    = m.AugMat(1, 3) ...
                                        + dot( m.RtHS.PLinVelEO(1, :, 1), m.RtHS.FrcONcRtt ) ...
                                        + dot( m.RtHS.PAngVelEB(1, :, 1), m.RtHS.MomBNcRtt );

    
    TmpVec  = p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.PAngVelEG(1, :, 2));


            m.AugMat(2, 2)   =  -dot( m.RtHS.PAngVelEL(1, :, 2), ...
                                                                m.RtHS.PMomLPRot(:, 2));

        
        m.AugMat(2, 3)        = dot( (m.RtHS.PAngVelEL(1, :, 2))', m.RtHS.MomLPRott) - GBoxTrq;
        
        m.AugMat(2, 2)    = m.AugMat(2, 2) ...
                                            + dot( m.RtHS.PAngVelEG(1, :, 2), TmpVec);

        m.AugMat(2, 3)        = m.AugMat(2, 3) ...
                                            - dot( m.RtHS.AngAccEGt, TmpVec);


    M = m.AugMat(1:2,1:2);
    F = m.AugMat(1:2,3);

end

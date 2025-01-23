% -------------------------------------------------------------------------
%
% This function is used to calculate the forces and moments 
% stored in other states
%
% -------------------------------------------------------------------------
%
% Inputs - Misc.:       m.CoordSys.c1       -
%                       m.CoordSys.d2       - 
%                       m.CoordSys.z2       -       
%                       m.RtHS.rOU          -  
%                       m.RtHS.rVP          -
%                       m.RtHS.AngVelEG     -
%       	            m.RtHS.AngAccEGt    -
%                       m.RtHS.AngAccENt    - 
%                       m.RtHS.LinAccEUt    -                       
%
% -------------------------------------------------------------------------
%
% Outputs:              m.RtHS.MomLPRott    -
%                       m.RtHS.FrcONcRtt    -
%                       m.RtHS.MomBNcRtt    - 
%
% -------------------------------------------------------------------------
function m = CalculateForcesMoments_v1(p, m, u)
    
    %% (Partial) Forces/Moments at the teeter pin (P) due to the rotor
%     for L = 1:p.DOFs.NActvDOF
%         for I = 1:3
%             m.RtHS.PFrcPRot(I,L)    = u.PFrcPRot(I, L);
%             m.RtHS.PMomLPRot(I,L)   = u.PMomLPRot(I, L);
%         end
%     end

    %% Forces/Moments at the teeter pin (P) due to the rotor
    for I = 1:3
        m.RtHS.FrcPRott(I,1)        = u.FrcPRott(I, 1);
        m.RtHS.MomLPRott(I,1)       = u.MomLPRott(I, 1);
    end
    
    %% (Partial) Froces/Moments at the nacelle (N) due to the furling structure
%     m.RtHS.PFrcVGnRt            = m.RtHS.PFrcPRot;
%     m.RtHS.PMomNGnRt            = m.RtHS.PMomLPRot;  
%     for I = 1:p.DOFs.NActvDOF
%         TmpVec                                  = cross( m.RtHS.rVP, m.RtHS.PFrcPRot(:, p.DOFs.SrtPS(I)) );
%         m.RtHS.PMomNGnRt(:, p.DOFs.SrtPS(I))    = m.RtHS.PMomNGnRt(:, p.DOFs.SrtPS(I)) + TmpVec';
%     end     % I - Active DOFs
%     if p.DOF_Flag(p.DOF_GeAz)
%         m.RtHS.PMomNGnRt(:, p.DOF_GeAz)         = m.RtHS.PMomNGnRt(:, p.DOF_GeAz) ...
%                                                     - (p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.PAngVelEG(1, :, p.DOF_GeAz) ))';
%     end
    
    %% Forces/Moments at at the nacelle (N) due to the furling structure
    % Influences of the furling structure are neglected
    TmpVec3                     = cross( m.RtHS.rVP, m.RtHS.FrcPRott );
    TmpVec                      = p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.AngVelEG );
    TmpVec5                     = cross ( -m.RtHS.AngVelEG, TmpVec );

    m.RtHS.FrcVGnRtt            = m.RtHS.FrcPRott;
    m.RtHS.MomNGnRtt            = m.RtHS.MomLPRott + TmpVec3' + TmpVec5' - (p.GenIner.*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.AngAccEGt ))';

    %% (Partial) Froces/Moments at the base plate (B) / yaw bearing (O) due to nacelle, generator, and rotor
%     m.RtHS.PFrcONcRt            = m.RtHS.PFrcVGnRt;
%     m.RtHS.PMomBNcRt            = m.RtHS.PMomNGnRt;           % Reasonable, since moment due to the tail can be neglected
%     for I = 1:p.DOFs.NPUE
%         TmpVec1                             = -p.NacMass*m.RtHS.PLinVelEU(1, :, p.DOFs.PUE(I));
%         TmpVec2                             = cross( m.RtHS.rOU, TmpVec1 );
%         
%         m.RtHS.PFrcONcRt(:, p.DOFs.PUE(I))  = m.RtHS.PFrcONcRt(:, p.DOFs.PUE(I)) + TmpVec1';
%         m.RtHS.PMomBNcRt(:, p.DOFs.PUE(I))  = m.RtHS.PMomBNcRt(:, p.DOFs.PUE(I)) ...
%                                                 + (TmpVec2 - p.Nacd2Iner*m.CoordSys.d2*dot( m.CoordSys.d2, m.RtHS.PAngVelEN(1, :, p.DOFs.PUE(I)) ))';
%     end     % I - Active DOFs that contribute to the linear accelerations of the nacelle CoM

    %% Forces/Moments at the yaw bearing (O)
    TmpVec1                     = (-p.NacMass*( p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEUt ))';
    TmpVec2                     = (cross( m.RtHS.rOU, TmpVec1 ))';
    m.RtHS.FrcONcRtt            = m.RtHS.FrcVGnRtt + TmpVec1;
    m.RtHS.MomBNcRtt            = m.RtHS.MomNGnRtt + TmpVec2 ...
                                    - (p.Nacd2Iner*m.CoordSys.d2*dot( m.CoordSys.d2, m.RtHS.AngAccENt ))';

end

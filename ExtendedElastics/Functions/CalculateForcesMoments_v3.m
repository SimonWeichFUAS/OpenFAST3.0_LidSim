% -------------------------------------------------------------------------
%
% This function is used to calculate the forces and moments 
% stored in other states
%
% -------------------------------------------------------------------------
%
% Inputs - Misc.:       m.CoordSys.c1       - req. 
%                       m.CoordSys.d2       - not req. after simplifications
%                       m.CoordSys.z2       - not req. after simplifications      
%                       m.RtHS.rOU          - not req. after simplifications 
%                       m.RtHS.rVP          - req.
%                       m.RtHS.AngVelEG     - req.
%       	            m.RtHS.AngAccEGt    - not req. after simplifications
%                       m.RtHS.AngAccENt    - not req. after simplifications
%                       m.RtHS.LinAccEUt    - not req. after simplifications                      
%
% -------------------------------------------------------------------------
%
% Outputs:              m.RtHS.MomLPRott    - req.      
%                       m.RtHS.FrcONcRtt    - req.
%                       m.RtHS.MomBNcRtt    - req.
%
% -------------------------------------------------------------------------
function [FrcONcRtt,MomBNcRtt] = CalculateForcesMoments_v3(p, x, FrcPRott, MomLPRott)
    
    %% (Partial) Forces/Moments at the teeter pin (P) due to the rotor
%     for L = 1:p.DOFs.NActvDOF
%         for I = 1:3
%             m.RtHS.PFrcPRot(I,L)    = u.PFrcPRot(I, L);
%             m.RtHS.PMomLPRot(I,L)   = u.PMomLPRot(I, L);
%         end
%     end

    %% Forces/Moments at the teeter pin (P) due to the rotor
%     for I = 1:3
%         m.RtHS.FrcPRott(I,1)        = u.FrcPRott(I, 1);
%         m.RtHS.MomLPRott(I,1)       = u.MomLPRott(I, 1);
%     end
%     
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
    
    %% Forces/Moments at at the nacelle (N) due to the furling structure - original
    % Influences of the furling structure are neglected
%     TmpVec3                     = cross( m.RtHS.rVP, m.RtHS.FrcPRott );
%     TmpVec                      = p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.AngVelEG );
%     TmpVec5                     = cross( -m.RtHS.AngVelEG, TmpVec );
% 
%     m.RtHS.FrcVGnRtt            = m.RtHS.FrcPRott;
%     m.RtHS.MomNGnRtt            = m.RtHS.MomLPRott + TmpVec3' + TmpVec5' - (p.GenIner.*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.AngAccEGt ))';

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

    %% Forces/Moments at the yaw bearing (O) - original
%     TmpVec1                     = (-p.NacMass*( p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEUt ))'; 
%     TmpVec2                     = (cross( m.RtHS.rOU, TmpVec1 ))';
%     m.RtHS.FrcONcRtt            = m.RtHS.FrcVGnRtt + TmpVec1;
%     m.RtHS.MomBNcRtt            = m.RtHS.MomNGnRtt + TmpVec2 ...
%                                     - (p.Nacd2Iner*m.CoordSys.d2*dot( m.CoordSys.d2, m.RtHS.AngAccENt ))';

    %% Check 
%     global CFM1 CFM2 CFM3 CFM4 CFM5 CFM6 CFM7
%         CFM1    = [CFM1, (p.Nacd2Iner*m.CoordSys.d2*dot( m.CoordSys.d2, m.RtHS.AngAccENt ))' ];                 % 1st: 0; 2nd: 0; 3rd: 0             
%         CFM2    = [CFM2, (cross( m.RtHS.rOU, TmpVec1 ))'];                                                      % 1st: 0; 2nd: 0; 3rd: mean=2.9829e+07 (could work!)
%         CFM3    = [CFM3, (-p.NacMass*( p.Gravity*m.CoordSys.z2 + m.RtHS.LinAccEUt ))' ];                        % 1st: mean=-3.2120e+01 (try =0); 2nd: mean=-6.3439e+06 (could work!); 3rd: 0
%         CFM4    = [CFM4, (p.GenIner.*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.AngAccEGt ))' ];                  % 1st: ~0; 2nd: ~0; 3rd: 0
%         CFM5    = [CFM5, cross( -m.RtHS.AngVelEG, TmpVec )' ];                                                  % required!
%         CFM6    = [CFM6, (p.GenIner*m.CoordSys.c1*dot( m.CoordSys.c1, m.RtHS.AngVelEG ))' ];                    % required!
%         CFM7    = [CFM7, cross( m.RtHS.rVP, m.RtHS.FrcPRott )' ];                                               % required!
   
    %% Forces/Moments at at the nacelle (N) due to the furling structure - simplified 
    % Influences of the furling structure are neglected
    TmpVec3                     = cross( ([1.8025e-02, 4.3494e+00, 0] + [-1.2026e+01, 1.3162e+00, 0]), FrcPRott );
    TmpVec                      = p.GenIner*[9.9406e-01, -1.0880e-01, 0]*dot( [9.9406e-01, -1.0880e-01, 0], (x.qdt(p.DOF_TFA1).*[0; 0; -1.4857e-02] + x.qdt(p.DOF_GeAz).*[9.9406e-01; -1.0880e-01; 0]) );
    TmpVec5                     = cross( -(x.qdt(p.DOF_TFA1).*[0; 0; -1.4857e-02] + x.qdt(p.DOF_GeAz).*[9.9406e-01; -1.0880e-01; 0]), TmpVec );

%     FrcVGnRtt            = FrcPRott;
    MomNGnRtt            = MomLPRott + TmpVec3' + TmpVec5';

    %% Forces/Moments at the yaw bearing (O) - simplified
    TmpVec1                     = [0;   -6.3439e+06;    0]; 
    TmpVec2                     = [0;   0;              2.9829e+07];
    FrcONcRtt            = FrcPRott + TmpVec1;
    MomBNcRtt            = MomNGnRtt + TmpVec2;
    
end

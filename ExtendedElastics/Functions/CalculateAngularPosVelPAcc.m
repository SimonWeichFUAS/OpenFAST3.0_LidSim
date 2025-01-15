% -------------------------------------------------------------------------
%
% This function is used to calculate the angular positions, velocities, 
% and partial accelerations stored in other states
%
% -------------------------------------------------------------------------
function m = CalculateAngularPosVelPAcc(p, x, m)
    
    m.RtHS.PAngVelEX(1, :, :)           = 0;
    m.RtHS.AngVelEX                     = [0, 0, 0];
    m.RtHS.AngAccEXt                    = 0;            % Reasonable, since we dont consider platform motion

    m.RtHS.PAngVelEB(:,:,:)             = 0;
    m.RtHS.PAngVelEB(1, :, p.DOF_TFA1)  = -p.TwrFASF(1,p.TTopNode,2).*m.CoordSys.a3;
    m.RtHS.PAngVelEB(2, :, p.DOF_TFA1)  = cross( m.RtHS.AngVelEX, m.RtHS.PAngVelEB(1, :, p.DOF_TFA1) );

    m.RtHS.AngVelEB                     = m.RtHS.AngVelEX + x.qdt(p.DOF_TFA1)*m.RtHS.PAngVelEB(1, :, p.DOF_TFA1);      % Note: we only consider TFA1
    m.RtHS.AngAccEBt                    = m.RtHS.AngAccEXt + x.qdt(p.DOF_TFA1).*m.RtHS.PAngVelEB(2, :, p.DOF_TFA1);

    m.RtHS.PAngVelEN(1, :, :)           = m.RtHS.PAngVelEB(1, :, :);
    m.RtHS.PAngVelEN(2, :, :)           = m.RtHS.PAngVelEB(2, :, :);
    m.RtHS.AngVelEN                     = m.RtHS.AngVelEB;              % Reasonable, since we dont consier yaw
    m.RtHS.AngAccENt                    = m.RtHS.AngAccEBt; 

    m.RtHS.PAngVelER(1, :, :)           = m.RtHS.PAngVelEN(1, :, :);
    m.RtHS.PAngVelER(2, :, :)           = m.RtHS.PAngVelEN(2, :, :);
    m.RtHS.AngVelER                     = m.RtHS.AngVelEN;              % Reasonable, since we dont consider Furl
    m.RtHS.AngAccERt                    = m.RtHS.AngAccENt;
    
    m.RtHS.PAngVelEL(1, :, :)           = m.RtHS.PAngVelER(1, :, :);
    m.RtHS.PAngVelEL(2, :, :)           = m.RtHS.PAngVelER(2, :, :);
    m.RtHS.PAngVelEL(1, :, p.DOF_GeAz)  = m.CoordSys.c1;
    m.RtHS.PAngVelEL(2, :, p.DOF_GeAz)  = cross( m.RtHS.AngVelER, m.RtHS.PAngVelEL(1, :, p.DOF_GeAz) );
    m.RtHS.AngVelEL                     = m.RtHS.AngVelER + x.qdt(p.DOF_GeAz)*m.RtHS.PAngVelEL(1, :, p.DOF_GeAz);       % We are ignoring the drivetrain DOF at this point
    AngAccELt                           = m.RtHS.AngAccERt + x.qdt(p.DOF_GeAz)*m.RtHS.PAngVelEL(2, :, p.DOF_GeAz);

    m.RtHS.PAngVelEH(1, :, :)           = m.RtHS.PAngVelEL(1, :, :);
    m.RtHS.PAngVelEH(2, :, :)           = m.RtHS.PAngVelEL(2, :, :);
    m.RtHS.AngVelEH                     = m.RtHS.AngVelEL;
    m.RtHS.AngAccEHt                    = AngAccELt;

    m.RtHS.PAngVelEG(1, :, :)           = m.RtHS.PAngVelER(1, :, :);
    m.RtHS.PAngVelEG(1, :, p.DOF_GeAz)  = p.GBRatio.*m.CoordSys.c1;
    m.RtHS.PAngVelEG(2, :, p.DOF_GeAz)  = cross( m.RtHS.AngVelER, m.RtHS.PAngVelEG(1, :, p.DOF_GeAz));
    m.RtHS.AngVelEG                     = m.RtHS.AngVelER + x.qdt(p.DOF_GeAz)*m.RtHS.PAngVelEG(1, :, p.DOF_GeAz);
    m.RtHS.AngAccEGt                    = m.RtHS.AngAccERt + x.qdt(p.DOF_GeAz).*m.RtHS.PAngVelEG(2, :, p.DOF_GeAz);

end

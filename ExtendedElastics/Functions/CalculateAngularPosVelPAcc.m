function m = CalculateAngularPosVelPAcc(p, x, m)
    
    m.RtHS.PAngVelEB(:,:,:)             = 0;
    m.RtHS.PAngVelEB(1, :, p.DOF_TFA1)  = -p.TwrFASF(1,p.TTopNode,2).*m.CoordSys.a3;

    m.RtHS.PAngVelEL(1, :, p.DOF_GeAz)  = m.CoordSys.c1;

    m.RtHS.PAngVelEG(1, :, p.DOF_GeAz)  = p.GBRatio.*m.CoordSys.c1;
    
    m.RtHS.AngVelEX                     = [0, 0, 0];        % Check again, no platform motion?
    m.RtHS.AngAccEXt                    = 0;

    m.RtHS.AngVelEB                     = cross( m.RtHS.AngVelEX, m.RtHS.PAngVelEB(1, :, p.DOF_TFA1));      % Note: we only consider TFA1
    m.RtHS.AngAccEBt                    = m.RtHS.AngAccEXt + x.qdt(p.DOF_TFA1).*m.RtHS.PAngVelEB(1, :, p.DOF_TFA1);

    m.RtHS.AngVelEN                     = m.RtHS.AngVelEB;          % Reasonable, sincewe dont consier yaw
    m.RtHS.AngAccENt                    = m.RtHS.AngAccEBt; 

    m.RtHS.AngVelER                     = m.RtHS.AngVelEN;          % Reasonable, since we dont consider Furl
    m.RtHS.AngAccERt                    = m.RtHS.AngAccENt;

    m.RtHS.PAngVelEG(2, :, p.DOF_GeAz)  = cross( m.RtHS.AngVelER, m.RtHS.PAngVelEG(1, :, p.DOF_GeAz));
    m.RtHS.AngAccEGt                    = m.RtHS.AngAccERt + x.qdt(p.DOF_GeAz).*m.RtHS.PAngVelEG(2, :, p.DOF_GeAz);

end

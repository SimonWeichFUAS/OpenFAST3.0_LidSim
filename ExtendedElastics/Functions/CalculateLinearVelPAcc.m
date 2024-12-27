function m = CalculateLinearVelPAcc(p, x, m)
    
    % Initializations
    m.RtHS.LinAccEOt(:,:)   = 0;
    m.RtHS.LinAccETt(:,:)   = 0;
    
    %%
    m.RtHS.PLinVelEO(:,:,:)             = 0;        % Reasonable, since RtHSdat%PLinVelEZ(:,:,:) = 0 for this 2DOF-case
    m.RtHS.PLinVelEO(1, :, p.DOF_TFA1)  = m.CoordSys.a1 - ( p.AxRedTFA(1,1,p.TTopNode)*x.qt(p.DOF_TFA1) ).*m.CoordSys.a2;
    m.RtHS.PLinVelEO(2, :, p.DOF_TFA1)  = [0,0,0] - ( p.AxRedTFA(1,1,p.TTopNode)*x.qdt(p.DOF_TFA1) ).*m.CoordSys.a2;
    m.RtHS.LinAccEOt                    = x.qdt(p.DOF_TFA1)*m.RtHS.PLinVelEO(2, :, p.DOF_TFA1);
    
    %%
    m.RtHS.PLinVelEU(:,:,:)             = m.RtHS.PLinVelEO(:,:,:);
    
    %%
    for J = 1:p.TwrNodes+1
        m.RtHS.PLinVelET(:,:,:,J)               = 0;        % Reasonable, since RtHSdat%PLinVelEZ(:,:,:) = 0 for this 2DOF-case
        m.RtHS.PLinVelET(1, :, p.DOF_TFA1, J)   = p.TwrFASF(1,J,1)*m.CoordSys.a1 - (p.AxRedTFA(1,1,J)*x.qt(p.DOF_TFA1)).*m.CoordSys.a2;
        m.RtHS.PLinVelET(2, :, p.DOF_TFA1, J)   = [0,0,0] - ( p.AxRedTFA(1,1,J)*x.qdt(p.DOF_TFA1) ).*m.CoordSys.a2;
        m.RtHS.LinAccETt(:,J)                   = x.qdt(p.DOF_TFA1).*m.RtHS.PLinVelET(2, :, p.DOF_TFA1, J);
    end

end

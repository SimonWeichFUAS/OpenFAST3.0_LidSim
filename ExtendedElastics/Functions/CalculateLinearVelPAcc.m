function m = CalculateLinearVelPAcc(p, x, m)
    
    % Initializations
    m.RtHS.LinAccEOt(:,:)   = 0;
    m.RtHS.LinAccETt(:,:)   = 0;
    
    %% (Partial) Linear velocity of the base plate (point O) in the inertia frame (body E - earth) + 1st time derivative
    m.RtHS.PLinVelEO(:,:,:)             = 0;        % Reasonable, since RtHSdat%PLinVelEZ(:,:,:) = 0 for this 2DOF-case
    m.RtHS.PLinVelEO(1, :, p.DOF_TFA1)  = m.CoordSys.a1 - ( p.AxRedTFA(1,1,p.TTopNode)*x.qt(p.DOF_TFA1) ).*m.CoordSys.a2;
    m.RtHS.PLinVelEO(2, :, p.DOF_TFA1)  = [0,0,0] - ( p.AxRedTFA(1,1,p.TTopNode)*x.qdt(p.DOF_TFA1) ).*m.CoordSys.a2;
    m.RtHS.LinAccEOt                    = x.qdt(p.DOF_TFA1)*m.RtHS.PLinVelEO(2, :, p.DOF_TFA1);
    
    %% (Partial) Linear velocity of the nacelle CoM (point U) in the inertia frame (body E - earth)
    m.RtHS.PLinVelEU(:,:,:)             = m.RtHS.PLinVelEO(:,:,:); 
    
    %% (Partial) Linear velocity of a point on the furl axis (point V) in the inertia frame (body E - earth)
    m.RtHS.PLinVelEV(:,:,:)             = m.RtHS.PLinVelEO(:,:,:);
    
    %% (Partial) Linear velocity of the teeter pin (point P) in the inertia frame (body E - earth)
    m.RtHS.PLinVelEP(:,:,:)             = m.RtHS.PLinVelEV(:,:,:);
    for I = 1:p.DOFs.NPR
        TmpVec0                                 = cross(m.RtHS.PAngVelER(1, :, p.DOFs.PR(I)), m.RtHS.rVP);

        m.RtHS.PLinVelEP(1, :, p.DOFs.PR(I))    = TmpVec0 + m.RtHS.PLinVelEP(1, :, p.DOFs.PR(I));
    end

    %% (Partial) Linear velocity of the apex of rotation (point Q) in the inertia frame (body E - earth)
    m.RtHS.PLinVelEQ(:,:,:)             = m.RtHS.PLinVelEP(:,:,:);

    %% (Partial) Linear velocity of the hub CoM (point C) in the inertia frame (body E - earth)
    m.RtHS.PLinVelEC(:,:,:)             = m.RtHS.PLinVelEQ(:,:,:);

    %% (Partial) Linear velocity of a point on a blade (point S) in the inertia frame (body E - earth)
    for K = 1:p.NumBl
        for J = 1:p.TipNode+1
            m.RtHS.PLinVelES(:,:,:,J,K)                    = m.RtHS.PLinVelEQ(:,:,:);
            for I = 1:p.DOFs.NPH
                TmpVec0         = cross( m.RtHS.PAngVelEH(1, :, p.DOFs.PH(I)), m.RtHS.rQS(:, J, K));

                m.RtHS.PLinVelES(1,:,p.DOFs.PH(I),J,K)           = m.RtHS.PLinVelES(1,:,p.DOFs.PH(I),J,K) + TmpVec0;
            end     % I - All DOFs assiciated with the angular motion of the hub
        end     % J - Loop through the blade nodes
    end     % K - Number of blades
    

    %% (Partial) Linear velocity of a point on the tower (point T) in the inertia frame (body E - earth)
    for J = 1:p.TwrNodes+1
        m.RtHS.PLinVelET(:,:,:,J)               = 0;        % Reasonable, since RtHSdat%PLinVelEZ(:,:,:) = 0 for this 2DOF-case
        m.RtHS.PLinVelET(1, :, p.DOF_TFA1, J)   = p.TwrFASF(1,J,1)*m.CoordSys.a1 - (p.AxRedTFA(1,1,J)*x.qt(p.DOF_TFA1)).*m.CoordSys.a2;
        m.RtHS.PLinVelET(2, :, p.DOF_TFA1, J)   = [0,0,0] - ( p.AxRedTFA(1,1,J)*x.qdt(p.DOF_TFA1) ).*m.CoordSys.a2;
        m.RtHS.LinAccETt(:,J)                   = x.qdt(p.DOF_TFA1).*m.RtHS.PLinVelET(2, :, p.DOF_TFA1, J);
    end

end

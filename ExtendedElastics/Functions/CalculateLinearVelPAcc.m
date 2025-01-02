% -------------------------------------------------------------------------
%
% This function is used to calculate the linear velocities and 
% accelerations stored in other states
%
% -------------------------------------------------------------------------
function m = CalculateLinearVelPAcc(p, x, m)
    
    % Initializations
    m.RtHS.LinAccECt(:,:)   = 0.0;
    m.RtHS.LinAccEOt(:,:)   = 0.0;
    m.RtHS.LinAccESt(:,:,:) = 0.0;
    m.RtHS.LinAccETt(:,:)   = 0.0;
    m.RtHS.LinAccEUt(:,:)   = 0.0;

    EwNXrOU                 = cross( m.RtHS.AngVelEN, m.RtHS.rOU );
    EwHXrQC                 = cross( m.RtHS.AngVelEH, m.RtHS.rQC );
    
    %% (Partial) Linear velocity of the base plate (point O) in the inertia frame (body E - earth) + 1st time derivative
    m.RtHS.PLinVelEO(:,:,:)             = 0;        % Reasonable, since RtHSdat%PLinVelEZ(:,:,:) = 0 for this 2DOF-case
    m.RtHS.PLinVelEO(1, :, p.DOF_TFA1)  = m.CoordSys.a1 - ( p.AxRedTFA(1,1,p.TTopNode)*x.qt(p.DOF_TFA1) ).*m.CoordSys.a2;
    m.RtHS.PLinVelEO(2, :, p.DOF_TFA1)  = [0,0,0] - ( p.AxRedTFA(1,1,p.TTopNode)*x.qdt(p.DOF_TFA1) ).*m.CoordSys.a2;
    m.RtHS.LinAccEOt                    = x.qdt(p.DOF_TFA1)*m.RtHS.PLinVelEO(2, :, p.DOF_TFA1);
    
    %% (Partial) Linear velocity of the nacelle CoM (point U) in the inertia frame (body E - earth)
    m.RtHS.PLinVelEU(:,:,:)             = m.RtHS.PLinVelEO(:,:,:); 

    for I = 1:p.DOFs.NPN
        TmpVec0                                 = cross( m.RtHS.PAngVelEN(1, :, p.DOFs.PN(I)), m.RtHS.rOU );
        TmpVec1                                 = cross( m.RtHS.PAngVelEN(1, :, p.DOFs.PN(I)), EwNXrOU );
        TmpVec2                                 = cross( m.RtHS.PAngVelEN(2, :, p.DOFs.PN(I)), m.RtHS.rOU );

        m.RtHS.PLinVelEU(1, :, p.DOFs.PN(I))    = m.RtHS.PLinVelEU(1, :, p.DOFs.PN(I)) + TmpVec0;
        m.RtHS.PLinVelEU(2, :, p.DOFs.PN(I))    = m.RtHS.PLinVelEU(2, :, p.DOFs.PN(I)) + TmpVec1 + TmpVec2;

        m.RtHS.LinAccEUt                        = m.RtHS.LinAccEUt + x.qdt(p.DOFs.PN(I))*m.RtHS.PLinVelEU(2, :, p.DOFs.PN(I));
    end     % All DOFs associated with the angular motion of the nacelle
    
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
    m.RtHS.PLinVelEC(:,:,:)                     = m.RtHS.PLinVelEQ(:,:,:);
    for I = 1:p.DOFs.NPH
        TmpVec0                                 = cross( m.RtHS.PAngVelEH(1, :, p.DOFs.PH(I)), m.RtHS.rQC );
        TmpVec1                                 = cross( m.RtHS.PAngVelEH(1, :, p.DOFs.PH(I)), EwHXrQC );
        TmpVec2                                 = cross( m.RtHS.PAngVelEH(2, :, p.DOFs.PH(I)), m.RtHS.rQC);

        m.RtHS.PLinVelEC(1, :, p.DOFs.PH(I))    = m.RtHS.PLinVelEC(1, :, p.DOFs.PH(I)) + TmpVec0;
        m.RtHS.PLinVelEC(2, :, p.DOFs.PH(I))    = m.RtHS.PLinVelEC(2, :, p.DOFs.PH(I)) + TmpVec1 + TmpVec2;

        m.RtHS.LinAccECt                        = m.RtHS.LinAccECt + (x.qdt(p.DOFs.PH(I))*m.RtHS.PLinVelEC(2, :, p.DOFs.PH(I)))';
    end     % I - All DOFs associated with the angular motion of the hub

    %% (Partial) Linear velocity of a point on a blade (point S) in the inertia frame (body E - earth)
    for K = 1:p.NumBl
        for J = 1:p.TipNode+1
            EwHXrQS                                     = cross( m.RtHS.AngVelEH, m.RtHS.rQS(:,J,K) );
            % Since no blade DOFs are considered, this section is simplified accordingly
            LinVelHS                                    = [ 0, 0, 0 ];
            m.RtHS.PLinVelES(:,:,:,J,K)                 = m.RtHS.PLinVelEQ(:,:,:);
            for I = 1:p.DOFs.NPH
                TmpVec0         = cross( m.RtHS.PAngVelEH(1, :, p.DOFs.PH(I)), m.RtHS.rQS(:, J, K) );
                TmpVec1         = cross( m.RtHS.PAngVelEH(1, :, p.DOFs.PH(I)), EwHXrQS + LinVelHS );
                TmpVec2         = cross( m.RtHS.PAngVelEH(2, :, p.DOFs.PH(I)), m.RtHS.rQS(:, J, K) );

                m.RtHS.PLinVelES(1,:,p.DOFs.PH(I),J,K)          = m.RtHS.PLinVelES(1,:,p.DOFs.PH(I),J,K) + TmpVec0;
                m.RtHS.PLinVelES(2,:,p.DOFs.PH(I),J,K)          = m.RtHS.PLinVelES(2,:,p.DOFs.PH(I),J,K) + TmpVec1 + TmpVec2;
                
                m.RtHS.LinAccESt(:,J,K)                         = m.RtHS.LinAccESt(:,J,K) + (x.qdt(p.DOFs.PH(I))*m.RtHS.PLinVelES(2,:,p.DOFs.PH(I),J,K))';
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

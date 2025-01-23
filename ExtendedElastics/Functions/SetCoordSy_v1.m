% -------------------------------------------------------------------------
%
% This function is used to define the internal coordinate systems 
% for this particular step
%
% -------------------------------------------------------------------------
%
% Inputs - Misc.:       None
%
% -------------------------------------------------------------------------
%
% Outputs:              m.CoordSys.a1       -
%                       m.CoordSys.a2       -
%                       m.CoordSys.a3       -
%                       m.CoordSys.c1       - 
%                       m.CoordSys.d1       -
%                       m.CoordSys.d2       - 
%                       m.CoordSys.d3       -
%                       m.CoordSys.rf1      -
%                       m.CoordSys.rf2      - 
%                       m.CoordSys.rf3      - 
%                       m.CoordSys.z2       -
%
% -------------------------------------------------------------------------
function m = SetCoordSy(p, x, m, u)
    
    % Inertial frame coordinate system
    m.CoordSys.z1   = [1, 0, 0];        % Vector/direction z1 (=  xi from the IEC coord. system)
    m.CoordSys.z2   = [0, 1, 0];        % Vector/direction z2 (=  zi from the IEC coord. system)
    m.CoordSys.z3   = [0, 0, 1];        % Vector/direction z3 (= -yi from the IEC coord. system)

    % Tower base / platform coordinate system
    TransMat        = SmllRotTrans(0, 0, -0);       % Since not platform DOFs are considered, the angles can be set to 0 in this case

    m.CoordSys.a1   = TransMat(1,1)*m.CoordSys.z1 + TransMat(1,2)*m.CoordSys.z2 + TransMat(1,3)*m.CoordSys.z3;
    m.CoordSys.a2   = TransMat(2,1)*m.CoordSys.z1 + TransMat(2,2)*m.CoordSys.z2 + TransMat(2,3)*m.CoordSys.z3;
    m.CoordSys.a3   = TransMat(3,1)*m.CoordSys.z1 + TransMat(3,2)*m.CoordSys.z2 + TransMat(3,3)*m.CoordSys.z3;
    
    % Tower-top / base plate coordinate system
    ThetaFA         =  -p.TwrFASF(1, p.TTopNode,2)*x.qt(p.DOF_TFA1);
    ThetaSS         = 0;

    TransMat        = SmllRotTrans(ThetaSS, ThetaFA, 0);
    m.CoordSys.b1   = TransMat(1,1)*m.CoordSys.a1 + TransMat(2,1)*m.CoordSys.a2 + TransMat(3,1)*m.CoordSys.a3;
    m.CoordSys.b2   = TransMat(1,2)*m.CoordSys.a1 + TransMat(2,2)*m.CoordSys.a2 + TransMat(3,2)*m.CoordSys.a3;
    m.CoordSys.b3   = TransMat(1,3)*m.CoordSys.a1 + TransMat(2,3)*m.CoordSys.a2 + TransMat(3,3)*m.CoordSys.a3; 
 
    % Nacelle / yaw coordinate system
    m.CoordSys.d1   = m.CoordSys.b1;    % Reasonable, since DOF_Yaw is not included
    m.CoordSys.d2   = m.CoordSys.b2;
    m.CoordSys.d3   = m.CoordSys.b3;

    % Rotor-furl coordinate system 
    m.CoordSys.rf1  = m.CoordSys.d1;    % Reasonable, since Rotor-Furl is not included
    m.CoordSys.rf2  = m.CoordSys.d2;
    m.CoordSys.rf3  = m.CoordSys.d3;

    % Shaft coordinate system
    m.CoordSys.c1   =  1*p.CShftTilt.*m.CoordSys.rf1 + p.SShftTilt.*m.CoordSys.rf2 + 0*m.CoordSys.rf3;
%     m.CoordSys.c2   = -1*p.SShftTilt.*m.CoordSys.rf1 + p.CShftTilt.*m.CoordSys.rf2 + 0*m.CoordSys.rf3;
%     m.CoordSys.c3   =  0.*m.CoordSys.rf1             + 0                          + 1*m.CoordSys.rf3;

    % Azimuth coordinate system
%     CAzimuth        = cos(x.qt(p.DOF_GeAz));
%     SAzimuth        = sin(x.qt(p.DOF_GeAz));

%     m.CoordSys.e1   =  m.CoordSys.c1;
%     m.CoordSys.e2   =  CAzimuth*m.CoordSys.c2 + SAzimuth*m.CoordSys.c3;
%     m.CoordSys.e3   = -SAzimuth*m.CoordSys.c2 + CAzimuth*m.CoordSys.c3;
    
    % Teeter coordinate system
%     m.CoordSys.f1   = m.CoordSys.e1;
%     m.CoordSys.f2   = m.CoordSys.e2;
%     m.CoordSys.f3   = m.CoordSys.e3;

    % Hub coordniate system
%     m.CoordSys.g1   = m.CoordSys.f1;
%     m.CoordSys.g2   = m.CoordSys.f2;
%     m.CoordSys.g3   = m.CoordSys.f3;

%     for K = 1:p.NumBl
%         % Hub coordinate system (rotated)
%         gRotAng     = p.TwoPiNB*(K-1);
%         CgRotAng    = cos(gRotAng);
%         SgRotAng    = sin(gRotAng);
% 
%         g1Prime     =  m.CoordSys.g1;
%         g2Prime     =  CgRotAng*m.CoordSys.g2 + SgRotAng*m.CoordSys.g3;
%         g3Prime     = -SgRotAng*m.CoordSys.g2 + CgRotAng*m.CoordSys.g3;
% 
%         % Coned coordinate system
%         m.CoordSys.i1(K, :) = p.CosPreC(K)*g1Prime - p.SinPreC(K)*g3Prime;
%         m.CoordSys.i2(K, :) = g2Prime;
%         m.CoordSys.i3(K, :) = p.SinPreC(K)*g1Prime + p.CosPreC(K)*g3Prime;
% 
%         % Blade / pitched coordinate system
%         CosPitch            = cos(u.BlPitch);
%         SinPitch            = sin(u.BlPitch);
% 
%         m.CoordSys.j1(K, :) = CosPitch*m.CoordSys.i1(K,:) - SinPitch*m.CoordSys.i2(K,:);
%         m.CoordSys.j2(K, :) = SinPitch*m.CoordSys.i1(K,:) + CosPitch*m.CoordSys.i2(K,:);
%         m.CoordSys.j3(K, :) = m.CoordSys.i3(K,:);
%     end     % K - number of blades

end

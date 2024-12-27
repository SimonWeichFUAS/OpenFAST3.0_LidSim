% This function is used to define the internal coordinate systems for this particular step
function m = SetCoordSy(p, x, m)
    
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
    m.CoordSys.c2   = -1*p.SShftTilt.*m.CoordSys.rf1 + p.CShftTilt.*m.CoordSys.rf2 + 0*m.CoordSys.rf3;
    m.CoordSys.c3   =  0.*m.CoordSys.rf1             + 0                          + 1*m.CoordSys.rf3;
    

end

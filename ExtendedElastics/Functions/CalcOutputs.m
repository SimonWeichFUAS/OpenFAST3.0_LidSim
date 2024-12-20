%  Function for computing the output channels
function [m, y] = CalcOutputs(u, p, x, m)
    
    m.AllOuts(p.Q_GeAz)       = x.qt(p.DOF_GeAz);
    m.AllOuts(p.Q_TFA1)       = x.qt(p.DOF_TFA1);

    m.AllOuts(p.QD_GeAz)      = x.qdt(p.DOF_GeAz);
    m.AllOuts(p.QD_TFA1)      = x.qdt(p.DOF_TFA1);

    m.AllOuts(p.QD2_GeAz)     = m.qd2t(p.DOF_GeAz);
    m.AllOuts(p.QD2_TFA1)     = m.qd2t(p.DOF_TFA1);

end

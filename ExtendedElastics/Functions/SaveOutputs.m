% This function saves the simulation outputs
function m = SaveOutputs(iStep, p, m)

    % Save outputs
    m.Results.qt(p.DOF_TFA1, iStep + 1)     = m.AllOuts(p.Q_TFA1);
    m.Results.qt(p.DOF_GeAz, iStep + 1)     = m.AllOuts(p.Q_GeAz);

    m.Results.qdt(p.DOF_TFA1, iStep + 1)    = m.AllOuts(p.QD_TFA1);
    m.Results.qdt(p.DOF_GeAz, iStep + 1)    = m.AllOuts(p.QD_GeAz);

    m.Results.qd2t(p.DOF_TFA1, iStep + 1)   = m.AllOuts(p.QD2_TFA1);
    m.Results.qd2t(p.DOF_GeAz, iStep + 1)   = m.AllOuts(p.QD2_GeAz);

end

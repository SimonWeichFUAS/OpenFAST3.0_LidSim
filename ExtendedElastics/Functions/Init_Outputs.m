% This function initializes the structs for saving simulation outputs
function [m, y] = Init_Outputs(p, x, InputFileData, m)

    % Allocate structs
    m.Results.qt        = AllocStruct(p.NDOF, p.Tend/p.dt + 1);
    m.Results.qdt       = AllocStruct(p.NDOF, p.Tend/p.dt + 1);
    m.Results.qd2t      = AllocStruct(p.NDOF, p.Tend/p.dt + 1);

    y.RotSpeed  = AllocStruct(p.Tend/p.dt + 1);
    y.TTDspFA   = AllocStruct(p.Tend/p.dt + 1);
    
    % Save initial conditions
    m.Results.qt(p.DOF_TFA1, 1)     = x.qt(p.DOF_TFA1);
    m.Results.qt(p.DOF_GeAz, 1)     = x.qt(p.DOF_GeAz);

    m.Results.qdt(p.DOF_TFA1, 1)    = x.qdt(p.DOF_TFA1);
    m.Results.qdt(p.DOF_GeAz, 1)    = x.qdt(p.DOF_GeAz);

    m.Results.qd2t(p.DOF_TFA1, 1)   = 0;        % Probably needs a different approach
    m.Results.qd2t(p.DOF_GeAz, 1)   = 0;        % for initilizing the accelerations

end

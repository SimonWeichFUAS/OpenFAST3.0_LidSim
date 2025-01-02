% -------------------------------------------------------------------------
%
% This function initializes the continuous states 
%
% -------------------------------------------------------------------------
function x = Init_ContStates(p, InputFileData)
    
    % Allocate structs
    x.qt    = AllocStruct(p.NDOF);
    x.qdt   = AllocStruct(p.NDOF);
    
    % Generator motion
    x.qt(p.DOF_GeAz)    = InputFileData.Azimuth - p.AzimB1Up - pi/2;
    x.qt(p.DOF_GeAz)    = mod(x.qt(p.DOF_GeAz), 2*pi);
    x.qdt(p.DOF_GeAz)   = p.RotSpeed;

    % Tower motion
    x.qt(p.DOF_TFA1)    = InputFileData.TTDspFA;
    x.qdt(p.DOF_TFA1)   = 0;

end

% -------------------------------------------------------------------------
%
% This function extract the corresponding input data for the current
% simulation step and stage of the runge-kutta method from the provided
% data structs
%
% -------------------------------------------------------------------------
function u = GetInputData_v2(iStep, RK4_stage, u, p)

    iRow    = RK4_stage + (iStep-1)*4;
   
    % Extract data from ServoDyn
    u.BlPitch               = u.RawDataInp(iRow, 1);       % Assumes the use of CPC
    u.GenTrq                = u.RawDataInp(iRow, 2);    

    % Extract addidional data from ElastoDyn
    % Portion of the force at point P due to the rotor
    for I = 1:3
        iCol                = 908 + I;
        u.FrcPRott(I, 1)    = u.RawDataInp(iRow, iCol);
    end

    % Portion of the moment at point P due to the rotor
    for I = 1:3
        iCol                = 917 + I;
        u.MomLPRott(I, 1)   = u.RawDataInp(iRow, iCol);
    end
    
end

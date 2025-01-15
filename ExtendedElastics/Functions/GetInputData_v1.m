% -------------------------------------------------------------------------
%
% This function extract the corresponding input data for the current
% simulation step and stage of the runge-kutta method from the provided
% data structs
%
% -------------------------------------------------------------------------
function u = GetInputData_v1(iStep, RK4_stage, u, p)

    iRow    = RK4_stage + (iStep-1)*4;
   
    % Extract data from ServoDyn
    u.BlPitch           = u.RawDataInp(iRow, 1);       % Assumes the use of CPC
    u.GenTrq            = u.RawDataInp(iRow, 2);    

    % Extract addidional data from ElastoDyn
    % Partial force at point P due to the rotor
    for L = 1:p.DOFs.NActvDOF
        for I = 1:3
            iCol                                = 902 + I + (L-1)*3;
            u.PFrcPRot(I, L)                    = u.RawDataInp(iRow, iCol);
        end
    end

    % Portion of the force at point P due to the rotor
    for I = 1:3
        iCol                                    = 908 + I;
        u.FrcPRott(I, 1)                        = u.RawDataInp(iRow, iCol);
    end

    % Partial moment at point P due to the rotor
    for L = 1:p.DOFs.NActvDOF
        for I = 1:3
            iCol                                = 911 + I + (L-1)*3;
            u.PMomLPRot(I, L)                   = u.RawDataInp(iRow, iCol);
        end
    end

    % Portion of the moment at point P due to the rotor
    for I = 1:3
        iCol                                    = 917 + I;
        u.MomLPRott(I, 1)                       = u.RawDataInp(iRow, iCol);
    end
    
end

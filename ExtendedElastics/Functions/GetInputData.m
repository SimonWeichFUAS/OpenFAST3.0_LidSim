% -------------------------------------------------------------------------
%
% This function extract the corresponding input data for the current
% simulation step and stage of the runge-kutta method from the provided
% data structs
%
% -------------------------------------------------------------------------
function u = GetInputData(iStep, RK4_stage, u, p)

    iRow    = RK4_stage + (iStep-1)*4;
   
    % Extract data from ServoDyn
    u.BlPitch           = u.RawDataInp(iRow, 1);       % Assumes the use of CPC
    u.GenTrq            = u.RawDataInp(iRow, 2);    

    % Extract addidional data from ElastoDyn
    % Aerodynamic forces
    for K = 1:p.NumBl
        for J = 1:p.BldNodes
            for I = 1:3
                iCol                            = 2 + I + (J-1)*3 + (K-1)*150;
                u.BladePtLoads(K).Force(I, J)   = u.RawDataInp(iRow, iCol);
            end     % I - Number of vector components
        end     % J - Number of blade nodes/elements
    end     % K - Numbber of blades
    
    % Aerodynamic moments
    for K = 1:p.NumBl
        for J = 1:p.BldNodes
            for I = 1:3
                iCol                            = 452 + I + (J-1)*3 + (K-1)*150;
                u.BladePtLoads(K).Moment(I, J)  = u.RawDataInp(iRow, iCol);
            end     % I - Number of vector components
        end     % J - Number of blade nodes/elements
    end     % K - Number of blades

end

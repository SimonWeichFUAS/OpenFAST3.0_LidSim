% -------------------------------------------------------------------------
%
% This function extract the corresponding input data for the current
% simulation step and stage of the runge-kutta method from the provided
% data structs
%
% -------------------------------------------------------------------------
function u = GetInputData(iStep, RK4_stage, u, p)

    % Extract relevant data
    u.BlPitch           = u.RawDataInp(iStep, 2);       % Assumes the use of CPC
    
    BaseIndexFrc        = 1;
    BaseIndexMmt        = 1;
    BlockSizeFrc        = 450;
    BlockSizeMmt        = 450;
    StartIndexFrc       = BaseIndexFrc + (iStep-1)*BlockSizeFrc*4 + (RK4_stage-1)*BlockSizeFrc;
    StartIndexMmt       = BaseIndexMmt + (iStep-1)*BlockSizeMmt*4 + (RK4_stage-1)*BlockSizeFrc;
    for K = 1:p.NumBl
        for J = 1:p.BldNodes
            u.BladePtLoads(K).Force(1, J)       = u.RawDataFrc(StartIndexFrc, 2);
            StartIndexFrc                       = StartIndexFrc + 1;
            u.BladePtLoads(K).Force(2, J)       = u.RawDataFrc(StartIndexFrc, 2);
            StartIndexFrc                       = StartIndexFrc + 1;
            u.BladePtLoads(K).Force(3, J)       = u.RawDataFrc(StartIndexFrc, 2);
            StartIndexFrc                       = StartIndexFrc + 1;

            u.BladePtLoads(K).Moment(1, J)      = u.RawDataMmt(StartIndexMmt, 2);
            StartIndexMmt                       = StartIndexMmt + 1;
            u.BladePtLoads(K).Moment(2, J)      = u.RawDataMmt(StartIndexMmt, 2);  
            StartIndexMmt                       = StartIndexMmt + 1;
            u.BladePtLoads(K).Moment(3, J)      = u.RawDataMmt(StartIndexMmt, 2);
            StartIndexMmt                       = StartIndexMmt + 1;
        end     % J - Number of blade nodes/elements
    end     % K - Number of blades

end

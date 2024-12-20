function u = GetInputData(iStep, RK4_stage, u, p)

    % Extract relevant input data
    BlockSize           = 2;
    BaseIndex           = 1;
    StartIndex          = BaseIndex + (iStep-1)*BlockSize*4 + (RK4_stage-1)*BlockSize;
    u                   = u.RawData(StartIndex:StartIndex+BlockSize-1, 2);

end

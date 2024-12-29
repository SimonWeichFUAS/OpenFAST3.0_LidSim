function u = GetInputData(iStep, RK4_stage, u, p)

    % Extract relevant data
    BlockSize           = 24;
    BaseIndex           = 1;
    StartIndex          = BaseIndex + (iStep-1)*BlockSize*4 + (RK4_stage-1)*BlockSize;
    u.ExtOutp           = u.RawData(StartIndex:StartIndex+BlockSize-1, 2);

    u.BlPitch           = u.RawDataInp(iStep, 2);       % Assumes the use of CPC

end

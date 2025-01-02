% -------------------------------------------------------------------------
%
% This function allocates the arrays stored 
% in the ExtendedOutsputs data structure
%
% -------------------------------------------------------------------------
function u = Init_u(ExtendedInputs, ExtendedForces, ExtendedMoments)
    
    u.RawDataInp    = ExtendedInputs;
    u.RawDataFrc    = ExtendedForces;
    u.RawDataMmt    = ExtendedMoments;

end

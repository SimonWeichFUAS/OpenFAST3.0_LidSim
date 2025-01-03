% -------------------------------------------------------------------------
%
% Function for the extraction of FAST-binary-outputs and
% additional/seperate output channels
%
% -------------------------------------------------------------------------
function [  Binaries, ...
            ExtendedInputs, ...
            ExtendedForces, ...
            ExtendedMoments ] = GetExtendedFASTOutputs( SimualationName, ...
                                                        ExtendedInputsName, ...
                                                        ExtendedForcesName, ...
                                                        ExtendedMomentsName )
    
    Binaries            = ReadFASTbinaryIntoStruct([SimualationName, '.outb']);
    ExtendedInputs      = readmatrix(ExtendedInputsName, 'Delimiter', ';');
    ExtendedForces      = readmatrix(ExtendedForcesName, 'Delimiter', ';');
    ExtendedMoments     = readmatrix(ExtendedMomentsName, 'Delimiter', ';');

end

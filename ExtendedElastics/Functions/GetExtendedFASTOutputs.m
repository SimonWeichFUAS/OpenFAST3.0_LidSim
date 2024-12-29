% Function for the extraction of FAST-binary-outputs and
% additional/seperate output channels
function [Binaries, ExtendedOutputs, ExtendedInputs] = GetExtendedFASTOutputs(SimualationName , ExtendedOutputsName, ExtendedInputsName)
    
    Binaries            = ReadFASTbinaryIntoStruct([SimualationName, '.outb']);
    ExtendedOutputs     = readmatrix(ExtendedOutputsName, 'Delimiter', ';');
    ExtendedInputs      = readmatrix(ExtendedInputsName, 'Delimiter', ';');

end

% Function for the extraction of FAST-binary-outputs and
% additional/seperate output channels
function [Binaries, ExtendedOutputs] = GetExtendedFASTOutputs(SimualationName , ExtendedOutputsName)
    
    Binaries            = ReadFASTbinaryIntoStruct([SimualationName, '.outb']);
    ExtendedOutputs     = readmatrix(ExtendedOutputsName, 'Delimiter', ';');

end

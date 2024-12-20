% Simple function to allocate the required structs 
% If only one input is given, create a 1xSize1 NaN vector
% If two inputs are given, create an Size2xSize1 NaN matrix
% If three inputs are given, create a Size3xSize2xSize1 NaN array
function OutStruct  = AllocStruct(Size1, Size2, Size3)

    if nargin == 1
        OutStruct = NaN(Size1, 1);
    elseif nargin == 2
        OutStruct = NaN(Size1, Size2);
    elseif nargin == 3
        OutStruct = NaN(Size1, Size2, Size3);
    else
        error('AllocStruct requires 1, 2, or 3 inputs.');
    end

end

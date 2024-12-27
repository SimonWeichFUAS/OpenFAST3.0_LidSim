% Simple function to allocate the required structs 
% If only one input is given, create a 1xSize1 NaN vector
% If two inputs are given, create an Size2xSize1 NaN matrix
% If three inputs are given, create a Size3xSize2xSize1 NaN array
function OutStruct  = AllocStruct(Size1, Size2, Size3, Size4)

    if nargin >= 1
        if isvector(Size1) && numel(Size1) > 1
            Dim1 = numel(Size1);
        else
            Dim1 = Size1;
        end
    else
        error('Size1 is required!');
    end

    if nargin >= 2
        if isvector(Size2) && numel(Size2) > 1
            Dim2 = numel(Size2); 
        else
            Dim2 = Size2; 
        end
    else
        Dim2 = 1;
    end

    if nargin == 3
        if isvector(Size3) && numel(Size3) > 1
            Dim3 = numel(Size3); 
        else
            Dim3 = Size3; 
        end
    else
        Dim3 = 1; 
    end

    if nargin == 4
        if isvector(Size4) && numel(Size4) > 1
            Dim4 = numel(Size4); 
        else
            Dim4 = Size4; 
        end
    else
        Dim4 = 1; 
    end

    OutStruct = NaN(Dim1, Dim2, Dim3, Dim4);

end

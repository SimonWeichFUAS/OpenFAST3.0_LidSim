% This function allocates the coordinate systems
function CoordSys = Alloc_CoordSys(p)

    % Allocate coordinate system arrays
    CoordSys.i1     = AllocStruct(p.NumBl, 3);
    CoordSys.i2     = AllocStruct(p.NumBl, 3);
    CoordSys.i3     = AllocStruct(p.NumBl, 3);

end

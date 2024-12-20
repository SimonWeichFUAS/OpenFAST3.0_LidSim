function RtHS = Alloc_RtHS(p)

    %% local variables
    dims    = 3;                % The position structs all must be allocated with a dimension for X,Y,and Z

    %% Positios
    RtHS.rT     = AllocStruct(dims, p.TwrNodes);

    %% Tower
    RtHS.rZT    = AllocStruct(dims, p.TwrNodes);

    %% Blades


end

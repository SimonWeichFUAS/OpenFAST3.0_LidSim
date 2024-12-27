function m = Alloc_RtHS(p, m)

    %% local variables
    dims    = 3;                % The position structs all must be allocated with a dimension for X,Y,and Z

    %% Positios
    m.RtHS.rT           = AllocStruct(dims, p.TwrNodes);

    %% Tower
    m.RtHS.rZT          = AllocStruct(dims, 0:p.TwrNodes);
    m.RtHS.LinAccETt    = AllocStruct(dims, 0:p.TwrNodes);   

    %% Blades

    %% Angular Velocities
    m.RtHS.PAngVelEB    = AllocStruct(1:2,dims,p.NDOF);

    %% Linear Velocities
    m.RtHS.PLinVelEO    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PLinVelET    = AllocStruct(1:2,dims,p.NDOF,1:p.TwrNodes);
    m.RtHS.PLinVelEU    = AllocStruct(1:2,dims,p.NDOF);
    
    %% Forces and Moments
    m.RtHS.PFrcONcRt    = AllocStruct(3, p.NDOF);

end

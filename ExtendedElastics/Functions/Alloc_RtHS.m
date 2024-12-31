function m = Alloc_RtHS(p, m)

    %% local variables
    dims    = 3;                % The position structs all must be allocated with a dimension for X,Y,and Z

    %% Positios
    m.RtHS.rT           = AllocStruct(dims, p.TwrNodes);
    m.RtHS.rS0S         = AllocStruct(dims, p.TipNode, p.NumBl);
    m.RtHS.rP0          = AllocStruct(dims, p.NumBl);
    m.RtHS.rQS          = AllocStruct(dims, p.TipNode+1, p.NumBl);

    %% Tower
    m.RtHS.rZT          = AllocStruct(dims, 0:p.TwrNodes);
    m.RtHS.LinAccETt    = AllocStruct(dims, 0:p.TwrNodes); 

    %% Blades

    %% Angular Velocities
    m.RtHS.PAngVelEB    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PAngVelEG    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PAngVelEX    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PAngVelER    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PAngVelEN    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PAngVelEL    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PAngVelEH    = AllocStruct(1:2,dims,p.NDOF);

    %% Linear Velocities
    m.RtHS.PLinVelEO    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PLinVelES    = AllocStruct(1:2,dims,p.NDOF,0:p.TipNode,p.NumBl);
    m.RtHS.PLinVelET    = AllocStruct(1:2,dims,p.NDOF,1:p.TwrNodes);
    m.RtHS.PLinVelEC    = AllocStruct(1:2,dims,p.NDOF);
    m.RtHS.PLinVelEU    = AllocStruct(1:2,dims,p.NDOF);

    %% Linear Accelerations
    m.RtHS.LinAccESt    = AllocStruct(dims,0:p.TipNode,p.NumBl);
    
    %% Forces and Moments
    m.RtHS.PFrcS0B      = AllocStruct(dims, p.NumBl, p.NDOF);
    m.RtHS.FrcS0Bt      = AllocStruct(dims, p.NumBl);
    m.RtHS.PMomH0B      = AllocStruct(dims, p.NumBl, p.NDOF);
    m.RtHS.MomH0Bt      = AllocStruct(dims, p.NumBl);
    m.RtHS.PFrcPRot     = AllocStruct(dims, p.NDOF);
    m.RtHS.PMomLPRot    = AllocStruct(dims, p.NDOF);
    m.RtHS.PFrcONcRt    = AllocStruct(dims, p.NDOF);
    m.RtHS.PMomBNcRt    = AllocStruct(dims, p.NDOF);

end

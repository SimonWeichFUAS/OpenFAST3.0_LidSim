% -------------------------------------------------------------------------
%
% This function initializes miscallanious part of the main script
%
% -------------------------------------------------------------------------
function m = Init_MiscOtherStates(p)
    
    % Allocate structs 
    m               = Alloc_RtHS(p);
    m.CoordSys      = Alloc_CoordSys(p);
    m.qd2t          = AllocStruct(p.NDOF);
    
    m.AllOuts       = AllocStruct(p.MaxOutputs);
    m.AllOuts(:)    = 0;

    m.AugMat        = AllocStruct(p.NDOF, p.NAug);
    m.AugMatTest    = AllocStruct(p.NDOF, p.NAug);
    m.SolnVec       = AllocStruct(p.DOFs.NActvDOF);
    m.AugMat_factor = AllocStruct(p.DOFs.NActvDOF, p.DOFs.NActvDOF);

end

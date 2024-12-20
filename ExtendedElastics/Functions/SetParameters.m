% This function sets the parameters, based on the data stored in InputFileData
function p = SetParameters(InputFileData)
   
    %% Set parameters from primary input file
    p.NumBl     = InputFileData.NumBl;
    p.TipRad    = InputFileData.TipRad;
    p.HubRad    = InputFileData.HubRad;
    p.TwrNodes  = InputFileData.TwrNodes;

    p.dt        = InputFileData.dt;
    p.Tend      = InputFileData.Tend;
    p.Gravity   = InputFileData.Gravity;
    p.OverHang  = InputFileData.OverHang;
    p.ShftGagL  = InputFileData.ShftGagL;
    p.TowerHt   = InputFileData.TowerHt;
    p.TowerBsHt = InputFileData.TowerBsHt;
    p.PtfmRefzt = InputFileData.PtfmRefzt;

    p.HubMass   = InputFileData.HubMass;
    p.HubIner   = InputFileData.HubIner;
    p.GenIner   = InputFileData.GenIner;
    p.NacMass   = InputFileData.NacMass;
    p.NacYIner  = InputFileData.NacYIner;
    p.YawBrMas  = InputFileData.YawBrMass;
    p.PtfmMass  = InputFileData.PtfmMass;
    p.GBoxEff   = InputFileData.GBoxEff;
    p.GBRatio   = InputFileData.GBoxRatio;
    
    p.TTDspFA   = InputFileData.TTDspFA;
    p.HubCM     = InputFileData.HubCM;
    p.AzimB1Up  = InputFileData.AzimB1Up;
    p.NacCMxn   = InputFileData.NacCMxn;
    p.NacCMyn   = InputFileData.NacCMyn;
    p.NacCMzn   = InputFileData.NacCMzn;

    % Initialize all of the DOF parameters
    p.NDOF                  = 2;                % This framework is currently only built for a 2DOF system (can be changed in the future)
    p.NAug                  = p.NDOF + 1;

    p.DOF_TFA1              = 1;                % Indexing will need to be changed if mor DOFs are added
    p.DOF_GeAz              = 2;

    p.Q_TFA1                = 1;               
    p.Q_GeAz                = 4;
    p.QD_TFA1               = 2;
    p.QD_GeAz               = 5;
    p.QD2_TFA1              = 3;
    p.QD2_GeAz              = 6;

    p.DOF_Flag              = AllocStruct(p.NDOF);   
    p.DOF_Flag(p.DOF_TFA1)  = InputFileData.TwFADOF1;
    p.DOF_Flag(p.DOF_GeAz)  = InputFileData.GenDOF;

    p.DOFs.NActvDOF         = 0;
    for iDOF = 1:p.NDOF 
        if p.DOF_Flag(iDOF)
            p.DOFs.NActvDOF                     = p.DOFs.NActvDOF + 1;
            p.DOFs.SrtPS(p.DOFs.NActvDOF)       = iDOF;
            p.DOFs.SrtPSNAUG(p.DOFs.NActvDOF)   = iDOF;
            p.DOFs.Diag(iDOF)                   = p.DOFs.NActvDOF;
        end
    end

    p.DOFs.SrtPSNAUG(p.DOFs.NActvDOF + 1 )           = p.NAug;
    
    % Calculate some indirect inputs
    p.TwoPiNB       = 2*pi/p.NumBl;
    p.rZT0zt        = p.TowerBsHt - p.PtfmRefzt;
    p.TwrFlexL      = p.TowerHt - p.TowerBsHt;
    p.BldFlexL      = p.TipRad - p.HubRad;
    
    p.CosPreC       = AllocStruct(p.NumBl);
    p.SinPreC       = AllocStruct(p.NumBl);
    p.CosPreC(:)    = cos(InputFileData.Precone);
    p.SinPreC(:)    = sin(InputFileData.Precone);
    p.CosDel3       = 1;
    p.SinDel3       = 0;

    p.AvgNrmTpRd    = p.TipRad*sum(p.CosPreC)/p.NumBl;
    p.ProjArea      = pi*(p.AvgNrmTpRd^2);
    p.RotSpeed      = rpm2radPs(InputFileData.RotSpeed);
    p.CShftTilt     = cos(InputFileData.ShftTilt);
    p.SShftTilt     = sin(InputFileData.ShftTilt);
    p.HubHt         = p.TowerHt + InputFileData.Twr2Shft + p.OverHang*p.SShftTilt;

    %% Outputs
    p.MaxOutputs    = 8;        % 4 GeAz / 4 TFA1
   

    %% Set blade and tower parameters

end

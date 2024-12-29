% This function sets the parameters, based on the data stored in InputFileData
function p = SetParameters(InputFileData)
   
    %% Set parameters from primary input file
    p.NumBl     = InputFileData.NumBl;
    p.BldNodes  = InputFileData.BldNodes;
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

    p.TipMass   = AllocStruct(p.NumBl);
    p.TipMass   = InputFileData.TipMass;
    
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
    p.DOF_GeAz              = 2;                % In the org. ElastoDyn: TFA1=7; GeAz=13

    p.Q_TFA1                = 1;               
    p.Q_GeAz                = 4;
    p.QD_TFA1               = 2;
    p.QD_GeAz               = 5;
    p.QD2_TFA1              = 3;
    p.QD2_GeAz              = 6;

    p.DOF_Flag              = AllocStruct(p.NDOF);   
    p.DOF_Flag(p.DOF_TFA1)  = InputFileData.TwFADOF1;
    p.DOF_Flag(p.DOF_GeAz)  = InputFileData.GenDOF;
    
    p.DOFs.NPCE             = 0;
    p.DOFs.NPTTE            = 0;
    p.DOFs.PSE              = AllocStruct(p.NumBl, p.NDOF);
    p.DOFs.NPSE             = AllocStruct(p.NumBl);
    p.DOFs.NPSE(:)          = 0;
    p.DOFs.NPUE             = 0;
    p.DOFs.NPR              = 0;
    p.DOFs.NPH              = 0;
    if p.DOF_Flag(p.DOF_TFA1) 
        p.DOFs.NPCE         = p.DOFs.NPCE + 1;
        p.DOFs.NPTTE        = p.DOFs.NPTTE + 1;
        p.DOFs.NPSE(:)      = p.DOFs.NPSE(:) + 1;
        p.DOFs.NPUE         = p.DOFs.NPUE + 1;
        p.DOFs.NPR          = p.DOFs.NPR + 1;
        p.DOFs.NPH          = p.DOFs.NPH +1;
        
        p.DOFs.PCE(p.DOFs.NPCE)         = p.DOF_TFA1;
        p.DOFs.PTTE(p.DOFs.NPUE)        = p.DOF_TFA1;
        p.DOFs.PSE(:, p.DOFs.NPSE(:))   = p.DOF_TFA1;
        p.DOFs.PUE(p.DOFs.NPUE)         = p.DOF_TFA1;
        p.DOFs.PR(p.DOFs.NPR)           = p.DOF_TFA1;
        p.DOFs.PH(p.DOFs.NPH)           = p.DOF_TFA1;
    end
    if p.DOF_Flag(p.DOF_GeAz)
        p.DOFs.NPCE         = p.DOFs.NPCE + 1;
        p.DOFs.NPSE(:)      = p.DOFs.NPSE(:) + 1;
        p.DOFs.NPH          = p.DOFs.NPH +1;
        
        p.DOFs.PCE(p.DOFs.NPCE)         = p.DOF_GeAz;
        p.DOFs.PSE(:, p.DOFs.NPSE(:))   = p.DOF_GeAz;
        p.DOFs.PH(p.DOFs.NPH)           = p.DOF_GeAz;
    end

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
    p.TTopNode      = p.TwrNodes + 2;
    p.TipNode       = p.BldNodes + 1;
    
    p.CosPreC       = AllocStruct(p.NumBl);
    p.SinPreC       = AllocStruct(p.NumBl);
    p.CosPreC(:)    = cosd(InputFileData.Precone);
    p.SinPreC(:)    = sind(InputFileData.Precone);
    p.CosDel3       = 1;
    p.SinDel3       = 0;

    p.AvgNrmTpRd    = p.TipRad*sum(p.CosPreC)/p.NumBl;
    p.ProjArea      = pi*(p.AvgNrmTpRd^2);
    p.RotSpeed      = rpm2radPs(InputFileData.RotSpeed);
    p.CShftTilt     = cosd(InputFileData.ShftTilt);
    p.SShftTilt     = sind(InputFileData.ShftTilt);
    p.HubHt         = p.TowerHt + InputFileData.Twr2Shft + p.OverHang*p.SShftTilt;

    %% Other parameters
    p.GBoxTrqConst  = 1.9786768E+07;        % Direct FAST output

    p.rVPxn         = 0; 
    p.rVPyn         = 0;
    p.rVPzn         = InputFileData.Twr2Shft;

    %% Outputs
    p.MaxOutputs    = 8;        % 3 GeAz_q + 1 GeAz_y + 3 TFA1_q + 1 TFA1_y
   

    %% Set blade and tower parameters
    p.BElmntMass                = AllocStruct(p.BldNodes, p.NumBl);
    p.BElmntMass(:,1)           = [ 7072.006;       6289.146;       5464.652;       4621.369;       4056.955;
                                    3646.652;       3003.224;       2464.755;       2022.612;       1695.428;
                                    1474.325;       1334.012;       1243.606;       1183.033;       1150.164;
                                    1137.041;       1126.012;       1111.325;       1091.997;       1072.229;
                                    1053.141;       1033.175;       1011.043;       986.6743;       962.0989;
                                    939.1528;       918.0357;       896.5394;       848.8683;       799.7932;
                                    748.3423;       695.2896;       644.1077;       605.9887;       567.6722;
                                    528.3086;       483.7771;       435.1243;       383.3909;       332.0824;
                                    279.2481;       234.3738;       195.8746;       162.3754;       134.9903;
                                    112.7140;       93.72598;       73.22883;       56.30964;       31.61844 ];
    p.BElmntMass(:,2)           = p.BElmntMass(:,1);
    p.BElmntMass(:,3)           = p.BElmntMass(:,1);

    p.RNodes                    = [ 1.170000,       3.510000,       5.849999,       8.190000,       10.53000, ...
                                    12.87000,       15.21000,       17.55000,       19.89000,       22.23000, ...
                                    24.57000,       26.91000,       29.25000,       31.59000,       33.93000, ...
                                    36.27000,       38.61000,       40.95000,       43.29000,       45.63000, ...
                                    47.97000,       50.31000,       52.65000,       54.99000,       57.33000, ...
                                    59.67000,       62.01000,       64.35000,       66.68999,       69.02999, ...
                                    71.36999,       73.70998,       76.04998,       78.38998,       80.72997, ...
                                    83.06997,       85.40997,       87.74996,       90.08996,       92.42995, ...
                                    94.76995,       97.10995,       99.44994,       101.7899,       104.1299, ...
                                    106.4699,       108.8099,       111.1499,       113.4899,       115.8299 ];
    
    p.AxRedTFA(1,1,:)           = [ 0.0000000E+00,  4.2345312E-07,  4.5842917E-06,  1.8516897E-05,  4.8419122E-05, ...
                                    1.0046216E-04,  1.8113300E-04,  2.9762340E-04,  4.5821589E-04,  6.7261787E-04, ...
                                    9.5218472E-04,  1.3099599E-03,  1.7604384E-03,  2.3189459E-03,  3.0005111E-03, ...
                                    3.8181194E-03,  4.7802739E-03,  5.8878614E-03,  7.1304617E-03,  8.4824441E-03, ...
                                    9.8995017E-03,  1.0617498E-02 ];

    p.TElmntMass                = AllocStruct(p.TwrNodes);
    p.TElmntMass(:)             = [ 67421.33,       65651.61,       63142.52,       60602.52,       57960.01, ...
                                    54734.87,       52182.66,       49425.29,       46795.02,       44065.73, ... 
                                    41337.62,       38517.70,       35632.43,       33348.83,       30673.76, ...
                                    28394.17,       26352.02,       24336.35,       24221.40,       25707.72 ];

    p.TwrFASF                   = AllocStruct(2,1:p.TTopNode,0:2);
    p.TwrFASF(1,:,1)            = [ 0.0000000E+00,  5.8609730E-04,  5.2409577E-03,  1.4466322E-02,  2.8195139E-02, ...
                                    4.6403594E-02,  6.9126248E-02,  9.6463919E-02,  0.1285843,      0.1657154, ...
                                    0.2081310,      0.2561304,      0.3100089,      0.3700225,      0.4363447, ...
                                    0.5090161,      0.5878868,      0.6725509,      0.7622752,      0.8559191, ...
                                    0.9518478,      1.000000 ];
    p.TwrFASF(1,:,2)            = [ 0.0000000E+00,  3.6181704E-04,  1.0749055E-03,  1.7753527E-03,  2.4682926E-03, ...
                                    3.1617659E-03,  3.8655952E-03,  4.5902566E-03,  5.3457562E-03,  6.1405040E-03, ...
                                    6.9801859E-03,  7.8666415E-03,  8.7967329E-03,  9.7612245E-03,  1.0743650E-02, ...
                                    1.1719198E-02,  1.2653571E-02,  1.3501873E-02,  1.4207475E-02,  1.4700900E-02, ...
                                    1.4898669E-02,  1.4856504E-02 ];

    p.DHNodes                   = AllocStruct(p.TwrNodes+2);
    p.DHNodes(:)                = [ 6.469300,       6.469300,       6.469300,       6.469300,       6.469300, ...
                                    6.469300,       6.469300,       6.469300,       6.469300,       6.469300, ...
                                    6.469300,       6.469300,       6.469300,       6.469300,       6.469300, ...
                                    6.469300,       6.469300,       6.469300,       6.469300,       6.469300, ...
                                    6.469300,       6.469300];

    p.KTFA                      = 2805922.0;
    p.CTFA                      = 3514.083;
    

end

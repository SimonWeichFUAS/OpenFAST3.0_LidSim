% This function provides the required inputs for running
% "RunFAST_ExtendedElastics.m" for the IEA15MW turbine
% Based on "ElastoDyn.dat"
function [InputFileData] = ElasticInputs_IEA15MW

%% Simulation Control
InputFileData.dt            = 0.01;                 % [s] Integration time step
InputFileData.Tend          = 30.0;                 % [s] Total simulation time (not included in original ElastoDyn.dat)

%% Environmental Conditions
InputFileData.Gravity       = 9.80665;              % [m/s^2] Gravitational acceleration

%% Degrees of Freedom
InputFileData.GenDOF        = true;                 % [flag] Generator DOF
InputFileData.TwFADOF1      = true;                 % [flag] First fore-aft tower bending-mode DOF

%% Initial Conditions
InputFileData.Azimuth       = 0.0;                  % [deg] Initial azimuth angle for blade 1
InputFileData.RotSpeed      = 7.56;                 % [rpm] Initial or fixed rotor speed
InputFileData.TTDspFA       = 0.2893;               % [m]   Initial fore-aft tower-top displacement

%% Turbine Configuration 
InputFileData.NumBl         = 3.0;                  % [-]   Number of blades
InputFileData.TipRad        = 120.97;               % [m]   The distance from the rotor apex to the blade tip
InputFileData.HubRad        = 3.97;                 % [m]   The distance from the rotor apex to the blade root
InputFileData.Precone       = -4.0;                 % [deg] Cone angle (assumed to be the same for every blade)
InputFileData.HubCM         = 0.0;                  % [m]   Distance from rotor apex to hub mass
InputFileData.UndSling      = 0.0;                  % [m]   Undersling length (unused for this configuration)
InputFileData.AzimB1Up      = 0.0;                  % [deg] Azimuth value to use for I/O when blade 1 points up
InputFileData.OverHang      = -12.097571763912535;  % [m]   Distance from yaw axis to rotor apex
InputFileData.ShftGagL      = 0.0;                  % [m]   Distance from rotor apex to shaft strain gages
InputFileData.ShftTilt      = -6.0;                 % [deg] Rotor shaft tilt angle
InputFileData.NacCMxn       = -4.720;               % [m]   Downwind distance from the tower-top to the nacelle CM
InputFileData.NacCMyn       = 0.0;                  % [m]   Lateral  distance from the tower-top to the nacelle CM
InputFileData.NacCMzn       = 4.275;                % [m]   Vertical distance from the tower-top to the nacelle CM
InputFileData.Twr2Shft      = 4.349459414248071;    % [m]   Vertical distance from the tower-top to the rotor shaft
InputFileData.TowerHt       = 144.386;              % [m]   Height of tower above ground level
InputFileData.TowerBsHt     = 15.0;                 % [m]   Height of tower base above ground level
InputFileData.PtfmRefzt     = 15.0;                 % [m]   Vertical distance from the ground level Vertical distance from the ground level

%% Mass an Inertia
InputFileData.TipMass(1)    = 0.0;                  % [kg]    Tip-brake mass for blade 1
InputFileData.TipMass(2)    = 0.0;                  % [kg]    Tip-brake mass for blade 2
InputFileData.TipMass(3)    = 0.0;                  % [kg]    Tip-brake mass for blade 3
InputFileData.HubMass       = 69360;                % [kg]    Hub mass
InputFileData.HubIner       = 973520;               % [kgm^2] Hub inertia about rotor axis 
InputFileData.GenIner       = 1836784;              % [kgm^2] Generator inertia about HSS
InputFileData.NacMass       = 646895;               % [kg]    Nacelle mass
InputFileData.NacYIner      = 24240914;             % [kgm^2] Nacelle inertia about yaw axis
InputFileData.YawBrMass     = 28280;                % [kg]    Yaw bearing mass
InputFileData.PtfmMass      = 0.0;                  % [kg]    Platform mass

%% Blade
InputFileData.BldNodes      = 50.0;                 % [-]     Number of blade nodes per blade

%% Drivetrain
InputFileData.GBoxEff       = 100.0;                % [prcnt] Gearbox efficiency
InputFileData.GBoxRatio     = 1.0;                  % [-]     Gearbox ratio

%% Tower
InputFileData.TwrNodes      = 20.0;                 % [-]   Number of tower nodes    


end                                                                 
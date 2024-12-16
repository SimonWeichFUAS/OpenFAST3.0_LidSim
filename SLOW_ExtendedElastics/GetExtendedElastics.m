%
%   This script can be used to replicate the calculations performed by the
%   ElastoDyn routine durning OpenFAST simulations.
%   
%   The forth-order runge-kutta mehtod is used to approximate the solutions
%   of the ODEs.
%
%   The resulting states are then compared with the actual outputs of the
%   Simulation.
%
%
%   v0 SW on 12/10/24: Setup of the general framework
%
%% Setup
clearvars; close all; clc

addpath(genpath('..\..\WetiMatlabFunctions'))
addpath(genpath('..\..\LidarAssistedControl'))

% Define directories/files and move the executable/map
WorkingDir              = [cd, '\'];
CompilationDir          = '..\build\bin\';
SimulationDir           = '..\..\LidarAssistedControl\Release\IEA15MW_01\';
SimualationName         = 'IEA-15-240-RWT-Monopile';

FASTexeFile             = 'openfast_x64.exe';
FASTmapFile             = 'MAP_x64.dll';
ExtendedElastics.Name   = 'ElastoDyn_Extended_Outputs.txt';

% Configuration parameters for ExtendedElastics
ExtendedElastics.Config.ActvDOFs            = [ "TFA1", "GEN" ];                        % Names of considered degrees of freedom
ExtendedElastics.Config.InitX               = [ 0.2893,  0               ];             % Initialization continous states
ExtendedElastics.Config.InitXdot            = [ 0,       rpm2radPs(7.56) ];             % Initialization continous differential states
ExtendedElastics.Config.TMax                = 30.0;                                     % [s] Simulation time
ExtendedElastics.Config.dt                  = 0.01;                                     % [s] Simulation time step

% Model parameters for ExtendedElastics
% Coordinate systems
ExtendedElastics.Model.CoordSys.xt          = [ 1.0,         0.0,          0.0 ];       % [-] Vector in xt-direction (Tower cs., see definiion in IEC)
ExtendedElastics.Model.CoordSys.zt          = [ 0.0,         1.0,          0.0 ];       % [-] Vector in zt-direction (Tower cs., see definiion in IEC)
ExtendedElastics.Model.CoordSys.xs          = [ 0.994063451, -0.108801906, 0.0 ];       % [-] Vector in xs-direction (Shaft cs., see definiion in IEC) 

% First tower fore-aft motion
ExtendedElastics.Model.TFA1                 = load('MassMatrix_Elements_TFA1.mat');     % Get pre-computed elements of the TFA1 mass-matrx 
ExtendedElastics.Model.TFA1.keT             = 2805921.5;                                % [kg/s^2]  tower equivalent bending stiffness (constant), from modidfied ElastoDyn console output
ExtendedElastics.Model.TFA1.ceT             = 3514.08349609;                            % [kg/s]	tower structual damping (constant), from modified ElastoDyn console output
ExtendedElastics.Model.TFA1.AxRedTFA        = 1.0617498E-02;                            % Axial-reduction term for the first fore-aft tower mode shape
ExtendedElastics.Model.TFA1.MassEff.YawBr   = load('MassEff_Elements_TFA1_YawBr.mat');  % Get pre-computed elements of the mass-effcts of the yaw bearing on the TFA1 motion; needs adjustments (too many elements!!!)
ExtendedElastics.Model.TFA1.MassEff.TwrNd   = load('MassEff_Elements_TFA1_TwrNd.mat');    % Get pre-computed elements of the mass-effects of he yaw bearing and tower nodes on the TFA1 motion   

% Generator motion
ExtendedElastics.Model.GEN                  = load('MassMatrix_Elements_GEN.mat');      % Get pre-computed elements of the GEN mass-matrix
ExtendedElastics.Model.GEN.GenTq            = 1.978676995881853e+07;                    % [Nm] Constant generator torque for above rated wind conditions; delcared as an input in ElastoDyn
ExtendedElastics.Model.GEN.GbRatio          = 1;                                        % [-]  Gearbox ratio   

%% Run Simulation using OpenFAST

if ~exist([SimualationName, '.outb'])
    copyfile([CompilationDir, FASTexeFile], [SimulationDir, FASTexeFile]);
    copyfile([CompilationDir, FASTmapFile], [SimulationDir, FASTmapFile]);

    cd(SimulationDir)
    
    dos([FASTexeFile, ' ', SimualationName, '.fst']);                                   % Run simulation
    movefile([SimualationName, '.outb'], [WorkingDir, SimualationName, '.outb'])        % Move binaries
    movefile(ExtendedElastics.Name, [WorkingDir, ExtendedElastics.Name])                % Move extended elastics
    
    delete(FASTexeFile)                                                                 % Delete executable
    delete(FASTmapFile)                                                                 % Delete map           
    
    cd(WorkingDir)
end

%% Read data from OpenFAST-Elastics        
Binaries            = ReadFASTbinaryIntoStruct([SimualationName, '.outb']);
tmp                 = readmatrix(ExtendedElastics.Name, 'Delimiter', ';');

% Get vector for simulation time
ExtendedElastics.Time       = [];

for iTime = 1:12:length(tmp)
    ExtendedElastics.Time   = [ExtendedElastics.Time; tmp(iTime,1)];
end

% Get data of active DOFs (for now, only the GEN-DOF is considered)
ExtendedElastics.Data       = NaN;

for nDOF = 1:length(ExtendedElastics.Config.ActvDOFs)
    fprintf('Reading data oth the following DOF: %s\n', ExtendedElastics.Config.ActvDOFs(nDOF));
    tmpData                 = NaN(ceil(length(tmp)/2),1);
    itmp                    = 1;

    for iData = nDOF:2:length(tmp)
        tmpData(itmp)       = tmp(iData,2);
        itmp                = itmp+1;
    end

    ExtendedElastics.Data(1:itmp-1, nDOF)  = tmpData(1:itmp-1);

    fprintf('Process complete!\n')
end

%% Replicate calculations of ElastoDyn using fouth-order Runge-Kutta

% Parameters
x0      = ExtendedElastics.Config.InitX;
x0_dot  = ExtendedElastics.Config.InitXdot;
dt      = ExtendedElastics.Config.dt;
TMax    = ExtendedElastics.Config.TMax;
nStep   = TMax/dt;
nx      = 2;
nq      = 2;

% Initialization
X               = NaN(nStep,nx);
X(1,:)          = x0;
X_dot           = NaN(nStep,nx);
X_dot(1,:)      = x0_dot;

x_NextStep      = x0;
x_dot_NextStep  = x0_dot;  

% Calculation loop
for iStep = 1:nStep
    x_ThisStep          = x_NextStep;
    x_dot_ThisStep      = x_dot_NextStep;


    [q1, q1_dot]        = CalcContStateDeriv(iStep, 1, X, X_dot, ExtendedElastics);
    k1_x                = dt * q1;
    k1_x_dot            = dt * q1_dot;
    
    [q2, q2_dot]        = CalcContStateDeriv(iStep, 2, X, X_dot, ExtendedElastics);
    k2_x                = dt * q2;
    k2_x_dot            = dt * q2_dot;
    
    [q3, q3_dot]        = CalcContStateDeriv(iStep, 3, X, X_dot, ExtendedElastics);
    k3_x                = dt * q3;
    k3_x_dot            = dt * q3_dot;
    
    [q4, q4_dot]        = CalcContStateDeriv(iStep, 4, X, X_dot, ExtendedElastics);
    k4_x                = dt * q4;
    k4_x_dot            = dt * q4_dot;

    x_NextStep          = x_ThisStep + (k1_x + 2*k2_x + 2*k3_x + k4_x)/6;
    x_dot_NextStep      = x_dot_ThisStep + (k1_x_dot + 2*k2_x_dot + 2*k3_x_dot + k4_x_dot)/6;
   
    X(iStep+1,:)        = x_NextStep;
    X_dot(iStep+1,:)    = x_dot_NextStep;
end

%% Plotting and comparing the replicated elastics with OpenFAST

% Allocate the required channels
Time            = Binaries.Time;
TTdispFA_FAST   = Binaries.TTDspFA;
Omega_FAST      = Binaries.RotSpeed;
TTdispFA_RepEl  = X(:,1);
Omega_RepEl     = X_dot(:,2);

% Compare the reults
n_plot = 4;
figure('Name','RepElvsFAST')

subplot(n_plot,1,1)
hold on; grid on; box on
plot(Time, Omega_FAST, 'o-')
plot(Time, radPs2rpm(Omega_RepEl), '.-')
ylabel({'RotSpeed'; '[rpm]'})
title('Comparison of replicated elastics and OpenFAST')
legend({'FAST', 'Replicated Elastics'})

subplot(n_plot,1,2)
hold on; grid on; box on
plot(Time, (Omega_FAST./radPs2rpm(Omega_RepEl) - 1)*100, '-o')
ylabel({'RotSpeed'; '[%]'})

subplot(n_plot,1,3)
hold on; grid on; box on
plot(Time, TTdispFA_FAST, 'o-')
plot(Time, TTdispFA_RepEl, '.-')
ylabel({'TTdispFA'; '[m]'})

subplot(n_plot,1,4)
hold on; grid on; box on
plot(Time, (TTdispFA_FAST./TTdispFA_RepEl - 1)*100, '-o')
ylabel({'TTdispFA'; '[%]'})
xlabel('time [s]')

%% Miscellaneous functions

% Calculate continous state derivatives
function [q, q_dot] = CalcContStateDeriv(iStep, nk, X, X_dot, Parameters)
    
    MassMatrix          = zeros(2,2);
    MassMatrix(1,1)     = Parameters.Model.TFA1.Masses_TFA1(iStep);
    MassMatrix(2,2)     = Parameters.Model.GEN.Masses_GEN(iStep);

    RtHdSd              = GetRtHdSd(iStep, nk, X, X_dot, Parameters);

    q                   = X_dot(iStep, :);     
    q_dot               = (MassMatrix\RtHdSd')';

end


% Get values of the right-hand-side of the ODE
function RtHdSd = GetRtHdSd(iStep, nk, X, X_dot, Parameters)
    
    % Parameters required for looping through the provied data sets
    nk_max  = 4;        % Number of Runge-Kutta stages
    nAx     = 3;        % Number of considered axes
    
    % TFA1 motion
    CoordSys_xt     = Parameters.Model.CoordSys.xt;
    CoordSys_zt     = Parameters.Model.CoordSys.zt;
    AxRedTFA1       = Parameters.Model.TFA1.AxRedTFA;
    k_eT            = Parameters.Model.TFA1.keT;
    c_eT            = Parameters.Model.TFA1.ceT;

    % Mass influences (partial forces)
    FrcMssEff_YawBr = Parameters.Model.TFA1.MassEff.YawBr.YawBr((iStep-1)*4+(nk-1)+1);
    FrcMssEff_TwrNd = Parameters.Model.TFA1.MassEff.TwrNd.TwrNd((iStep-1)*4+(nk-1)+1);

    DataIndex_TFA1  = (iStep - 1)*(nk_max * nAx) + (nk-1) * nAx + 1;
    FrcTFA1         = Parameters.Data(DataIndex_TFA1,1);

    % GEN motion
    CoordSys_xs = Parameters.Model.CoordSys.xs;
    GbRatio     = Parameters.Model.GEN.GbRatio;
    GenTq_HSS   = Parameters.Model.GEN.GenTq;
    GenTq_LSS   = GenTq_HSS * GbRatio;
    
    for iAx = 1:1:3
        DataIndex_GEN   = (iStep - 1) * (nk_max * nAx) + (nk - 1) * nAx + iAx;
        MomLPRott(iAx)  = Parameters.Data(DataIndex_GEN,2);
    end
    
    % Building the right-hand-side of the ODEs
    RtHdSd(1)   = FrcMssEff_YawBr;
    RtHdSd(1)   = RtHdSd(1) + FrcMssEff_TwrNd;
    RtHdSd(1)   = RtHdSd(1) - c_eT*X_dot(iStep,1) - k_eT*X(iStep,1);
    RtHdSd(1)   = RtHdSd(1) + FrcTFA1;
    RtHdSd(2)   = dot(MomLPRott, CoordSys_xs) - GenTq_LSS;
    
end































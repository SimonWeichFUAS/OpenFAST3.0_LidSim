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

copyfile([CompilationDir, FASTexeFile], [SimulationDir, FASTexeFile]);
copyfile([CompilationDir, FASTmapFile], [SimulationDir, FASTmapFile]);

% Configuration parameters for ExtendedElastics
ExtendedElastics.Config.ActvDOFs    = ["TFA1", "TFA1", "GEN", "GEN"];   % Names of considered degrees of freedom
ExtendedElastics.Config.InitX       = [0.2893,  0               ];      % Initialization continous states
ExtendedElastics.Config.InitXdot    = [0,       rpm2radPs(7.56) ];      % Initialization continous differential states
ExtendedElastics.Config.TMax        = 30;                                % [s] Simulation time
ExtendedElastics.Config.dt          = 0.01;                             % [s] Simulation time step

%% Run Simulation using OpenFAST

if ~exist([SimualationName, '.outb'])
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

for iTime = 1:16:length(tmp)
    ExtendedElastics.Time   = [ExtendedElastics.Time; tmp(iTime,1)];
end

% Get data of active DOFs
ExtendedElastics.Data       = NaN;

for nDOF = 1:length(ExtendedElastics.Config.ActvDOFs)
    fprintf('Reading data oth the following DOF: %s\n', ExtendedElastics.Config.ActvDOFs(nDOF));
    tmpData                 = NaN(ceil(length(tmp)/2),1);
    itmp                    = 1;

    for iData = nDOF:4:length(tmp)
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
    
    k1_x                = dt * ExtendedElastics.Data(iStep*4-3,1:2:4);
    k1_x_dot            = dt * ExtendedElastics.Data(iStep*4-3,2:2:4);

    k2_x                = dt * ExtendedElastics.Data(iStep*4-2,1:2:3);
    k2_x_dot            = dt * ExtendedElastics.Data(iStep*4-2,2:2:4);

    k3_x                = dt * ExtendedElastics.Data(iStep*4-1,1:2:3);
    k3_x_dot            = dt * ExtendedElastics.Data(iStep*4-1,2:2:4);

    k4_x                = dt * ExtendedElastics.Data(iStep*4,1:2:3);
    k4_x_dot            = dt * ExtendedElastics.Data(iStep*4,2:2:4);

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







































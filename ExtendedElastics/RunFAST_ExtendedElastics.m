%
% Main script for performing elastic simulations based on the
% ElastoDyn-Module of OpenFAST
%
%% Setup

clearvars; close all; clc

addpath(genpath('Model'))
addpath(genpath('Functions'))
addpath(genpath('..\..\WetiMatlabFunctions'))
addpath(genpath('..\..\LidarAssistedControl'))


% Define directories/files 
WorkingDir              = [cd, '\'];
CompilationDir          = '..\build\bin\';
SimulationDir           = '..\..\LidarAssistedControl\Release\IEA15MW_01\';
SimualationName         = 'IEA-15-240-RWT-Monopile';
ExtendedInputsName      = 'ElastoDyn_Extended_Inputs.txt';
ExtendedForcesName      = 'ElastoDyn_Extended_Forces.txt';
ExtendedMomentsName     = 'ElastoDyn_Extended_Moments.txt';

FASTexeFile             = 'openfast_x64.exe';
FASTmapFile             = 'MAP_x64.dll';

% Optional flags
PlotResults             = true;     % [true/false] Flag for plotting/comparing simulation results

%% Run simulation using OpenFAST

if ~exist([SimualationName, '.outb'])
    copyfile([CompilationDir, FASTexeFile], [SimulationDir, FASTexeFile]);              % Copy executable
    copyfile([CompilationDir, FASTmapFile], [SimulationDir, FASTmapFile]);              % Copy map

    cd(SimulationDir)
    
    dos([FASTexeFile, ' ', SimualationName, '.fst']);                                   % Run simulation
    movefile([SimualationName, '.outb'], [WorkingDir, SimualationName, '.outb'])        % Move binaries

    if exist(ExtendedInputsName)
        movefile(ExtendedInputsName, [WorkingDir, ExtendedInputsName])                  % Move extended inputs
    end

    if exist(ExtendedForcesName)
        movefile(ExtendedForcesName, [WorkingDir, ExtendedForcesName])                  % Move extended forces
    end

    if exist(ExtendedMomentsName)
        movefile(ExtendedMomentsName, [WorkingDir, ExtendedMomentsName])                % Move extended moments
    end
    
    delete(FASTexeFile)                                                                 % Delete executable
    delete(FASTmapFile)                                                                 % Delete map           
    
    cd(WorkingDir)
end

%% Main routine for running elastic simulations

% Initialization routine
[   Binaries, ...
    ExtendedInputs, ...
    ExtendedForces, ...
    ExtendedMoments ]       = GetExtendedFASTOutputs(   SimualationName , ...
                                                        ExtendedInputsName, ...
                                                        ExtendedForcesName, ...
                                                        ExtendedMomentsName);
InputFileData               = ElasticInputs_IEA15MW;
p                           = SetParameters(InputFileData);
x                           = Init_ContStates(p, InputFileData);
m                           = Init_MiscOtherStates(p, x, InputFileData);
u                           = Init_u(p, x, InputFileData, m, ExtendedInputs, ...
                                                             ExtendedForces, ...
                                                             ExtendedMoments);
[m, y]                      = Init_Outputs(p, x, InputFileData, m);

% Simulation routine
for iStep = 1:p.Tend/p.dt
    
    [u, x, m]   = UpdateStates(iStep, u, p, x, m);
    m           = CalcOutputs(u, p, x, m);
    [m, y]      = SaveOutputs(iStep, p, m, y);

end



%% Plot results

if PlotResults
    
    % Allocate channels
    Time            = Binaries.Time;
    Omega_FAST      = Binaries.RotSpeed;
    Omega_ExEl      = y.RotSpeed;
    TTdispFA_FAST   = Binaries.TTDspFA;
    TTdispFA_ExEl   = m.Results.qt(p.DOF_TFA1,:)';
    
    % Display results
    n_plot = 4;
    figure('Name','ExElvsFAST')

    subplot(n_plot,1,1)
    hold on; grid on; box on
    plot(Time, Omega_FAST, 'o-')
    plot(Time, radPs2rpm(Omega_ExEl), '.-')
    ylabel({'RotSpeed'; '[rpm]'})
    title('Comparison of ExtendedElastics and OpenFAST')
    legend({'FAST', 'ExEl'})

    subplot(n_plot,1,2)
    hold on; grid on; box on
    plot(Time, (Omega_FAST-radPs2rpm(Omega_ExEl))./p.RotSpeed, '-o')
    ylabel({'RotSpeed'; '[%]'})

    subplot(n_plot,1,3)
    hold on; grid on; box on
    plot(Time, TTdispFA_FAST, 'o-')
    plot(Time, TTdispFA_ExEl, '.-')
    ylabel({'TTdispFA'; '[m]'})

    subplot(n_plot,1,4)
    hold on; grid on; box on
    plot(Time, (TTdispFA_FAST-TTdispFA_ExEl)./p.TTDspFA, '-o')    
    ylabel({'TTdispFA'; '[%]'})
    xlabel('time [s]')

end

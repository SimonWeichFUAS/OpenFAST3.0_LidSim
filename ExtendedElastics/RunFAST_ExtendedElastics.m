%
%   [Description]
%   Main script for performing elastic simulations based on the
%   ElastoDyn-Module of OpenFAST
%
% -------------------------------------------------------------------------
%
%   [Version]
%   v4d2  -     Simplification of the augmented (mass) matrix   - SW    
%                                                               - 01/15/25
%
% -------------------------------------------------------------------------
%
%   [Notes]
%   The most recent version of OpenFAST according to this repository needs
%   to be compiled and stored inside of the "build" folder
% 
%   The two repositories "WetiMatlabFunctions" and "LidarAssistedControl"
%   need to exist inside of the partend-directory of this repository
%
%   A parameter description can be found in: 
%   "modules\elastodyn\src\ElastoDyn_Registry.txt"
%
% -------------------------------------------------------------------------
%% Setup

clearvars; close all; clc

% global F21 F22 F23 M21 M22
% global F11 F12 F13 F14 F15 M11 M12 M13 M14
% global CFM1 CFM2 CFM3 CFM4 CFM5 CFM6 CFM7
% global CLV1
% global CAV1 CAV2 CAV3
% global CPS1 CPS2 CPS3 CPS4
% global SCS1

addpath(genpath('Model'))
addpath(genpath('Functions'))
addpath(genpath('..\..\WetiMatlabFunctions'))
addpath(genpath('..\..\LidarAssistedControl'))


% Define directories/files 
WorkingDir              = [cd, '\'];
CompilationDir          = '..\build\bin\';
SimulationDir           = '..\..\LidarAssistedControl\Release\IEA15MW_01\';
SimualationName         = 'IEA-15-240-RWT-Monopile';
ExtendedOutputsName     = 'output_data.txt';

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

    if exist(ExtendedOutputsName)
        movefile(ExtendedOutputsName, [WorkingDir, ExtendedOutputsName])                % Move output data
    end

    delete(FASTexeFile)                                                                 % Delete executable
    delete(FASTmapFile)                                                                 % Delete map           
    
    cd(WorkingDir)
end

%% Main routine for running elastic simulations

% Initialization routine
[   Binaries, ...
    ExtendedOutputs]        = GetExtendedFASTOutputs(   SimualationName , ...
                                                        ExtendedOutputsName );
InputFileData               = ElasticInputs_IEA15MW;
p                           = SetParameters(InputFileData);
x                           = Init_ContStates(p, InputFileData);
m                           = Init_MiscOtherStates(p);
u                           = Init_u(ExtendedOutputs);
[m, y]                      = Init_Outputs(p, x, m);

% Simulation routine
cdt                         = clock;                            
fprintf(['Simulation started on %02d-%s-%04d' ...
            ' at %02d:%02d:%02d using Matlab...\n'], ...
            cdt(3),datestr(cdt,'mmm'),cdt(1),cdt(4),cdt(5),round(cdt(6)))
tStartTimer                 = tic;

n_t = p.Tend/p.dt;
for iStep = 1:n_t
    
    [u, x, m]   = UpdateStates(iStep, u, p, x, m);
    m           = CalcOutputs(p, x, m);
    [m, y]      = SaveOutputs(iStep, p, m, y);

end

tEndTimer                   = toc(tStartTimer);
fprintf(['Simulation finished after %2.2f seconds!\n'], tEndTimer)

%% Plot results / Calculate MRE

if PlotResults
    
    % Allocate channels
    Time            = Binaries.Time(1:n_t+1);
    RotSpeed_FAST   = Binaries.RotSpeed(1:n_t+1);
    RotSpeed_ExEl   = y.RotSpeed;
    TTDspFA_FAST    = Binaries.TTDspFA(1:n_t+1);
    TTDspFA_ExEl    = y.TTDspFA;
    
    % Display ExElvsFAST comparison
    n_plot = 4;
    figure('Name','ExElvsFAST')

    subplot(n_plot,1,1)
    hold on; grid on; box on
    plot(Time, RotSpeed_FAST, 'o-')
    plot(Time, radPs2rpm(RotSpeed_ExEl), '.-')
    ylabel({'RotSpeed'; '[rpm]'})
    title('Comparison of ExtendedElastics and OpenFAST')
    legend({'FAST', 'ExEl'})

    subplot(n_plot,1,2)
    hold on; grid on; box on
    plot(Time, (RotSpeed_FAST-radPs2rpm(RotSpeed_ExEl))./p.RotSpeed, '-o')
    ylabel({'RotSpeed'; '[%]'})

    subplot(n_plot,1,3)
    hold on; grid on; box on
    plot(Time, TTDspFA_FAST, 'o-')
    plot(Time, TTDspFA_ExEl, '.-')
    ylabel({'TTdispFA'; '[m]'})

    subplot(n_plot,1,4)
    hold on; grid on; box on
    plot(Time, (TTDspFA_FAST-TTDspFA_ExEl)./p.TTDspFA, '-o')    
    ylabel({'TTdispFA'; '[%]'})
    xlabel('time [s]')

    % Calculate MRE of RotSpeed and TTDspFA
    mRelErr_RotSpeed        = mean((RotSpeed_FAST-radPs2rpm(RotSpeed_ExEl))./p.RotSpeed);
    mRelErr_TTDspFA         = mean((TTDspFA_FAST-TTDspFA_ExEl)./p.TTDspFA);

    fprintf(['The mean relative error of the rotor speed is: %26.6e\n'],               mRelErr_RotSpeed)
    fprintf(['The mean relative error of the tower-top displacement is: %15.6e\n'],    mRelErr_TTDspFA)

end

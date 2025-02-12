% -------------------------------------------------------------------------
%
%   [Description]
%   This script compares the result of the "simplified
%   aerodynamics"-routine with the expected results from previous
%   simulations ("RunFAST_ExtendedElastics").
%
% -------------------------------------------------------------------------
%% Setup
clearvars; clc

addpath(genpath('..\..\Model'))
addpath(genpath('..\..\Functions'))
addpath(genpath('..\..\..\..\WetiMatlabFunctions'))
addpath(genpath('..\..\..\..\LidarAssistedControl'))

%% Simplified Aerodynamics
% Get binaries
Binaries                    = ReadFASTbinaryIntoStruct('Tilt6deg_TFA1on_Cone4deg.outb');
ElasticAeros                = load('Tilt6deg_TFA1on_Cone4deg.mat');
Time                        = Binaries.Time;
Wind1VelX                   = Binaries.Wind1VelX;
RtVAvgxh                    = Binaries.RtVAvgxh;
Wind1VelX_diff              = diff(Wind1VelX);
RtVAvgxh_diff               = diff(RtVAvgxh);
TwrTopVel                   = Binaries.QD_TFA1;
RotSpeed                    = Binaries.RotSpeed;
BlPitch                     = Binaries.BldPitch1;

% Get model parameters
TipRad                      = 120.97;
AirDen                      = 1.225;
AeroLUT                     = load('PowerAndThrustCoefficientsIEA15MWMonopile_only2DOF_adjAeroDyn.mat');
ShiftParam_Wind1VelX        = 160;                  % Mean-WS-12ms: 160                                                       
ShiftParam_RtVAvgxh         = 12;                   % Mean-WS-12ms: 12
ModCase                     = 'RtVAvgxh';           % 'Wind1VelX' or 'RtVAvgxh'

% Alloc outputs
nStep                       = length(Time)-1;
FrcAeroX_SLOW               = NaN(1, nStep);
MmtAeroX_SLOW               = NaN(1, nStep);
MmtAeroY_SLOW               = NaN(1, nStep);

% Shifted wind vector
% Wind1VelX
Wind1VelX_shifted           = [Wind1VelX(ShiftParam_Wind1VelX:end);         Wind1VelX(1:ShiftParam_Wind1VelX-1)];
Wind1VelX_diff_shifted      = [Wind1VelX_diff(ShiftParam_Wind1VelX:end);    Wind1VelX_diff(1:ShiftParam_Wind1VelX-1)];
Wind1VelX_rel_shifted       = Wind1VelX_shifted - TwrTopVel;
Wind1VelX_rel_diff_shifted  = diff(Wind1VelX_rel_shifted);

% RtVAvgxh
RtVAvgxh_shifted            = [RtVAvgxh(ShiftParam_RtVAvgxh:end);           RtVAvgxh(1:ShiftParam_RtVAvgxh-1)];
RtVAvgxh_diff_shifted       = [RtVAvgxh_diff(ShiftParam_RtVAvgxh:end);      RtVAvgxh_diff(1:ShiftParam_RtVAvgxh-1)];

% Simulation loop
switch ModCase
    case 'Wind1VelX'
        for iStep   = 1:nStep

            v0                      = Wind1VelX_shifted(iStep);
            vRel                    = Wind1VelX_rel_shifted(iStep);
            TSR                     = rpm2radPs(RotSpeed(iStep))*TipRad / vRel;

            cP                      = interp2(AeroLUT.theta, AeroLUT.lambda, AeroLUT.c_P, deg2rad(BlPitch(iStep)), TSR);
            cT                      = interp2(AeroLUT.theta, AeroLUT.lambda, AeroLUT.c_T, deg2rad(BlPitch(iStep)), TSR);

            FrcAeroX_SLOW(iStep)    = 0.5 * AirDen * pi * TipRad^2 * cT * vRel^2;
            FrcAeroX_SLOW(iStep)    = FrcAeroX_SLOW(iStep) - 3.0370e+05;                    % Mean-WS-12ms: - 3.0370e+05

            MmtAeroX_SLOW(iStep)    = 0.5 * AirDen * pi * TipRad^3 * cP * vRel^2 / TSR;
            MmtAeroX_SLOW(iStep)    = MmtAeroX_SLOW(iStep) - 1.3051e+06;                    % Mean-WS-12ms: - 1.3051e+06
            
            MmtAeroY_SLOW(iStep)    = Wind1VelX_rel_diff_shifted(iStep);
            MmtAeroY_SLOW(iStep)    = MmtAeroY_SLOW(iStep)*6e8 - 5.0243e+05;                % Mean-WS-12ms: *6e8 - 5.0243e+05

        end     % iStep - Number of considered simulation steps

    case 'RtVAvgxh'
        for iStep   = 1:nStep

            v0                      = RtVAvgxh_shifted(iStep);
            TSR                     = rpm2radPs(RotSpeed(iStep))*TipRad / v0;

            cP                      = interp2(AeroLUT.theta, AeroLUT.lambda, AeroLUT.c_P, deg2rad(BlPitch(iStep)), TSR);
            cT                      = interp2(AeroLUT.theta, AeroLUT.lambda, AeroLUT.c_T, deg2rad(BlPitch(iStep)), TSR);

            FrcAeroX_SLOW(iStep)    = 0.5 * AirDen * pi * TipRad^2 * cT * v0^2;
            FrcAeroX_SLOW(iStep)    = FrcAeroX_SLOW(iStep) - 2.9055e+05;                    % Mean-WS-12ms: - 2.9055e+05
            
            MmtAeroX_SLOW(iStep)    = 0.5 * AirDen * pi * TipRad^3 * cP * v0^2 / TSR;
            MmtAeroX_SLOW(iStep)    = MmtAeroX_SLOW(iStep) - 1.0108e+06;                    % Mean-WS-12ms: - 1.0108e+06

            MmtAeroY_SLOW(iStep)    = RtVAvgxh_diff_shifted(iStep);
            MmtAeroY_SLOW(iStep)    = MmtAeroY_SLOW(iStep)*9e8 - 5.0243e+05;                % TFA1=off
                                                                                            % Mean-WS-12ms: *6e8 - 5.0243e+05
                                                                                            % TFA1=on
                                                                                            % Mean-WS-12ms: *9e8 - 5.0243e+05
      
        end
end

% Display results
figure
subplot(4,1,1)
hold on; grid on; box on;
plot(Time,              Wind1VelX,              LineWidth=1.5)

if strcmp('RtVAvgxh', ModCase)
    plot(Time,              RtVAvgxh,           LineWidth=1.5)
    plot(Time,              RtVAvgxh_shifted,   LineWidth=1.5)
    legend({'Wind1VelX', 'RtVAvgxh', 'RtVAvgxh\_shifted'})
else
    plot(Time,          Wind1VelX_shifted,      LineWidth=1.5)
    legend({'Wind1VelX', 'Wind1VelX\_shifted'})
end

ylabel({'Windspeed'; '[m/s]'})
title('Aerodynamic Loads - Tilt6deg - Cone4deg - TFA1on')

subplot(4,1,2)
hold on; grid on; box on;
plot(Time(1:end-1),     ElasticAeros.FrcPRottAero(1, 1:4:end), LineWidth=1.5)
plot(Time(1:end-1),     FrcAeroX_SLOW, LineWidth=1.5)
legend({'FAST', 'SLOW'})
ylabel({'FrcAeroX'; '[N]'})

subplot(4,1,3)
hold on; grid on; box on;
plot(Time(1:end-1),     ElasticAeros.MomLPRottAero(1, 1:4:end), LineWidth=1.5)
plot(Time(1:end-1),     MmtAeroX_SLOW, LineWidth=1.5)
ylabel({'MmtAeroX'; '[Nm]'})

subplot(4,1,4)
hold on; grid on; box on;
plot(Time(1:end-1),     ElasticAeros.MomLPRottAero(3, 1:4:end), LineWidth=1.5)
plot(Time(1:end-1),     MmtAeroY_SLOW, LineWidth=1.5)
ylabel({'MmtAeroY'; '[Nm]'})
xlabel('Time [s]')
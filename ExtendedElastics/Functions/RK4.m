% This function implements the fourth-order Runge-Kutta Method
function [x, m] = RK4(iStep, u, p, x, m)

    % First stage
    RK4_stage   = 1;
    [xdot, m]   = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage);
    k1.qt       = p.dt * xdot.qt;
    k1.qdt      = p.dt * xdot.qdt;

    % Second stage
    RK4_stage   = 2;
    [xdot, m]   = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage);
    k2.qt       = p.dt * xdot.qt;
    k2.qdt      = p.dt * xdot.qdt;

    % Third stage
    RK4_stage   = 3;
    [xdot, m]   = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage);
    k3.qt       = p.dt * xdot.qt;
    k3.qdt      = p.dt * xdot.qdt;

    % Fourth stage
    RK4_stage   = 4;
    [xdot, m]   = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage);
    k4.qt       = p.dt * xdot.qt;
    k4.qdt      = p.dt * xdot.qdt;

    % Approximate solutions
    x.qt        = x.qt + (k1.qt + 2*k2.qt + 2*k3.qt + k4.qt)/6;
    x.qdt       = x.qdt + (k1.qdt + 2*k2.qdt + 2*k3.qdt + k4.qdt)/6;
    
end

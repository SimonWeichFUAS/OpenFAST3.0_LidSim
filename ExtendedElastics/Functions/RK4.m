% -------------------------------------------------------------------------
%
% This function implements the fourth-order Runge-Kutta Method
%
% -------------------------------------------------------------------------
function [u, x, m] = RK4(iStep, u, p, x, m)

    % First stage
    RK4_stage       = 1;
    [u, xdot, m]    = CalcContStateDeriv(iStep, u, p, x, m, RK4_stage);
    k1.qt           = p.dt * xdot.qt;
    k1.qdt          = p.dt * xdot.qdt;

    x_tmp.qt        = x.qt  + 0.5*k1.qt;
    x_tmp.qdt       = x.qdt + 0.5*k1.qdt;

    % Second stage
    RK4_stage       = 2;
    [u, xdot, m]   = CalcContStateDeriv(iStep, u, p, x_tmp, m, RK4_stage);
    k2.qt           = p.dt * xdot.qt;
    k2.qdt          = p.dt * xdot.qdt;

    x_tmp.qt        = x.qt  + 0.5*k2.qt;
    x_tmp.qdt       = x.qdt + 0.5*k2.qdt;

    % Third stage
    RK4_stage       = 3;
    [u, xdot, m]    = CalcContStateDeriv(iStep, u, p, x_tmp, m, RK4_stage);
    k3.qt           = p.dt * xdot.qt;
    k3.qdt          = p.dt * xdot.qdt;
    
    x_tmp.qt        = x.qt  + k3.qt;
    x_tmp.qdt       = x.qdt + k3.qdt;

    % Fourth stage
    RK4_stage       = 4;
    [u, xdot, m]    = CalcContStateDeriv(iStep, u, p, x_tmp, m, RK4_stage);
    k4.qt           = p.dt * xdot.qt;
    k4.qdt          = p.dt * xdot.qdt;

    % Approximate solutions
    x.qt            = x.qt + (k1.qt + 2*k2.qt + 2*k3.qt + k4.qt)/6;
    x.qdt           = x.qdt + (k1.qdt + 2*k2.qdt + 2*k3.qdt + k4.qdt)/6;
    
end

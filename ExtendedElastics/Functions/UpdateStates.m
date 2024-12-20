% This function updates states
function [x, m] = UpdateStates(iStep, u, p, x, m)

    [x, m] = RK4(iStep, u, p, x, m);
    
    if x.qt(p.DOF_GeAz) >= 2*pi
        x.qt(p.DOF_GeAz) = x.qt(p.DOF_GeAz) -2*pi;
    end

end

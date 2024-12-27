% This function computes a transformation matrix 
% based on small rotational angles
function TransMat = SmllRotTrans(roll, yaw, pitch)
    
    % Approximation for small angles
    sx = sin(roll);
    sy = sin(pitch);
    sz = sin(yaw);
    cx = cos(roll);
    cy = cos(pitch);
    cz = cos(yaw);

    % Transformation matrix
    TransMat        = zeros(3, 3);
    TransMat(1, 1) = cy * cz;
    TransMat(1, 2) = -cz * sy * sx - sz * cx;
    TransMat(1, 3) = sz * sx - cz * sy * cx;
    TransMat(2, 1) = sz * cy;
    TransMat(2, 2) = cz * cx - sz * sy * sx;
    TransMat(2, 3) = -cz * sx - sz * sy * cx;
    TransMat(3, 1) = -sy;
    TransMat(3, 2) = cy * sx;
    TransMat(3, 3) = cy * cx;

end

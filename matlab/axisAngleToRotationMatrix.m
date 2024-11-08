function R = axisAngleToRotationMatrix(axis)
    % axisAngleToRotationMatrix Converts an axis-angle representation to a rotation matrix.
    %
    % Input:
    %   axis - 1x3 vector representing the axis-angle, where the magnitude of the vector
    %          is the rotation angle in radians.
    %
    % Output:
    %   R - 3x3 rotation matrix

    % Calculate the rotation angle as the magnitude of the axis vector
    theta = norm(axis);

    % Check for zero rotation
    if theta == 0
        R = eye(3); % Identity matrix for zero rotation
        return;
    end

    % Normalize the axis vector
    axis = axis / theta;

    % Rodrigues' rotation formula components
    K = [0 -axis(3) axis(2);
         axis(3) 0 -axis(1);
        -axis(2) axis(1) 0];

    % Compute the rotation matrix
    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K^2);
end
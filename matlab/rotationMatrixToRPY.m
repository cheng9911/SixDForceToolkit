function [roll, pitch, yaw] = rotationMatrixToRPY(R)
    % rotationMatrixToRPY Converts a rotation matrix to Roll-Pitch-Yaw (RPY) angles.
    %
    % Input:
    %   R - 3x3 rotation matrix
    %
    % Output:
    %   roll  - Rotation around X-axis (in radians)
    %   pitch - Rotation around Y-axis (in radians)
    %   yaw   - Rotation around Z-axis (in radians)

    % 检查旋转矩阵的有效性
    if abs(R(3,1)) ~= 1
        % 非奇异情况
        pitch = -asin(R(3,1));
        roll = atan2(R(3,2) / cos(pitch), R(3,3) / cos(pitch));
        yaw = atan2(R(2,1) / cos(pitch), R(1,1) / cos(pitch));
    else
        % 奇异情况，当 pitch = ±90°
        yaw = 0;  % 设置 yaw 为 0，任意值都可以
        if R(3,1) == -1
            pitch = pi / 2;
            roll = yaw + atan2(R(1,2), R(1,3));
        else
            pitch = -pi / 2;
            roll = -yaw + atan2(-R(1,2), -R(1,3));
        end
    end
end
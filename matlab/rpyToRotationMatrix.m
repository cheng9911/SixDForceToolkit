function R = rpyToRotationMatrix(roll, pitch, yaw)
    % rpyToRotationMatrix Converts Roll-Pitch-Yaw (RPY) angles to a rotation matrix.
    %
    % Input:
    %   roll  - Rotation around X-axis (in radians)
    %   pitch - Rotation around Y-axis (in radians)
    %   yaw   - Rotation around Z-axis (in radians)
    %
    % Output:
    %   R - 3x3 rotation matrix

    % Rotation matrix around the X-axis (roll)
    Rx = [1, 0, 0;
          0, cos(roll), -sin(roll);
          0, sin(roll), cos(roll)];

    % Rotation matrix around the Y-axis (pitch)
    Ry = [cos(pitch), 0, sin(pitch);
          0, 1, 0;
          -sin(pitch), 0, cos(pitch)];

    % Rotation matrix around the Z-axis (yaw)
    Rz = [cos(yaw), -sin(yaw), 0;
          sin(yaw), cos(yaw), 0;
          0, 0, 1];

    % Combined rotation matrix, applied in ZYX order
    R = Rz * Ry * Rx;
end
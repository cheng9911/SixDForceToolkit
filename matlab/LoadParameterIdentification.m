function LoadParameterIdentification(forces, poses)
    % forces: 六维力矩阵 (6x6)，每行表示一个力向量
    % poses: 6x3 矩阵，包含每个姿态的滚转角、俯仰角和偏航角
    
    n = size(poses, 1); % 获取姿态数量
    F = zeros(3 * n, 6); % 初始化 F 矩阵
    M = zeros(3 * n, 1); % 初始化 M 矩阵

    % 填充 F 和 M 矩阵
    for i = 1:n
        F_temp = [0, forces(i,3), -forces(i,2), 1, 0, 0;
                  -forces(i,3), 0, forces(i,1), 0, 1, 0;
                   forces(i,2), -forces(i,1), 0, 0, 0, 1];
        M_temp = forces(i, 4:6);
        
        F((3*i-2):(3*i), :) = F_temp; % 将 F_temp 填入 F 矩阵
        M((3*i-2):(3*i), :) = M_temp; % 将 M_temp 填入 M 矩阵
    end

    % 使用最小二乘法求解载荷质心
    A = inv((F' * F)) * (F' * M); % 计算最小二乘解
    error = norm(F * A - M)/6; % 计算误差
    disp(['质心计算误差: ', num2str(error)]); % 显示误差
    
    massx = A(1); % 质心 x 坐标
    massy = A(2); % 质心 y 坐标
    massz = A(3); % 质心 z 坐标
    k1 = A(4); % 扭矩偏移项 1
    k2 = A(5); % 扭矩偏移项 2
    k3 = A(6); % 扭矩偏移项 3

    % 计算质量、零点和偏移角度在世界坐标系和基坐标系中的值
    F = zeros(3 * n, 6); % 重新初始化 F 矩阵
    M = zeros(3 * n, 1); % 重新初始化 M 矩阵

    % 使用旋转后的姿态填充 F 和 M 矩阵
    for i = 1:n
        R_temp = rpyToRotationMatrix(poses(i, 1), poses(i, 2), poses(i, 3)); % 计算旋转矩阵
        R_temp = R_temp'; % 转置旋转矩阵
        F_temp = [R_temp(1, :), 1, 0, 0;
                  R_temp(2, :), 0, 1, 0;
                  R_temp(3, :), 0, 0, 1];
        M_temp = forces(i, 1:3);
        
        F((3*i-2):(3*i), :) = F_temp; % 将 F_temp 填入 F 矩阵
        M((3*i-2):(3*i), :) = M_temp; % 将 M_temp 填入 M 矩阵
    end

    % 使用最小二乘法求解载荷质量
    A = pinv((F' * F)) * (F' * M); % 计算最小二乘解
    error = norm(F * A - M)/6; % 计算误差
    disp(['质量和零漂计算误差: ', num2str(error)]); % 显示误差
    det_value = det(F' * F); % 计算矩阵的行列式
    disp(['矩阵的行列式: ', num2str(det_value)]); % 显示行列式值
    
    G = sqrt(A(1)^2 + A(2)^2 + A(3)^2); % 计算合力大小
    gravity = 9.81; % 假设重力加速度为 9.81 m/s^2
    mass = G / gravity; % 计算质量
    U = asind(-A(2) / G); % 计算 U 角度
    V = atand(-A(1) / A(3)); % 计算 V 角度
    ZeroForceX = A(4); % 零点力 X 分量
    ZeroForceY = A(5); % 零点力 Y 分量
    ZeroForceZ = A(6); % 零点力 Z 分量

    disp(['G: ', num2str(G)]); % 显示合力大小
    disp(['U: ', num2str(U)]); % 显示 U 角度
    disp(['V: ', num2str(V)]); % 显示 V 角度
    disp(['ZeroForceX: ', num2str(ZeroForceX)]); % 显示零点力 X 分量
    disp(['ZeroForceY: ', num2str(ZeroForceY)]); % 显示零点力 Y 分量
    disp(['ZeroForceZ: ', num2str(ZeroForceZ)]); % 显示零点力 Z 分量

    % 计算基于质心的扭矩偏移
    ZeroTorqueRoll = k1 - ZeroForceY * massz + ZeroForceZ * massy; % 滚转扭矩偏移
    ZeroTorquePitch = k2 - ZeroForceZ * massx + ZeroForceX * massz; % 俯仰扭矩偏移
    ZeroTorqueYaw = k3 - ZeroForceX * massy + ZeroForceY * massx; % 偏航扭矩偏移

    disp(['ZeroTorqueRoll: ', num2str(ZeroTorqueRoll)]); % 显示滚转扭矩偏移
    disp(['ZeroTorquePitch: ', num2str(ZeroTorquePitch)]); % 显示俯仰扭矩偏移
    disp(['ZeroTorqueYaw: ', num2str(ZeroTorqueYaw)]); % 显示偏航扭矩偏移
end
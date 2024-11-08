% 姿态数据（假设机械臂姿态轴角表示->rpy,默认底层算法PRY处理的，
% 假设力传感器和机械臂之间的偏转角为f_TCP = rpyToRotationMatrix(3.1415926,0,-3.141592/4)

clc,clear,close all;
disp('Hello, from SixDForceTool!');

    % Define the flange poses
    % 机械臂姿态数据
    six_poses = [
        0.0123974,-0.575365,0.69058,-2.90244,1.20223,-1.09227e-05;
        0.163479,-0.583128,0.665763,2.28035e-05,1.18628e-05,-2.35617;
        -0.0496656,-0.598473,0.764768,-1.75998,0.729006,-1.76002;
        0.0474273,-0.595513,0.766014,1.76,-0.729018,-1.75999;
        0.191334,-0.557764,0.718021,0.613939,-1.4822,-0.61397;
        -0.0102881,-0.622588,0.798573,-1.76001,-0.729027,1.75996
    ];
    % Define the six-dimensional force data
    % 机械臂末端力传感器数据
    sixDForce = [
       -4.04905,-5.81165,-23.0022,0.188509,-0.803967,-0.0391097;
        -3.23488,-4.88059,7.19731,-0.0434611,0.20479,-0.0529627;
       -18.9389,-5.92461,-8.28863,0.143034,-1.14555,-0.0931772;
        11.403,-5.58718,-8.03096,0.115094,0.463372,0.00958173;
        -3.68731,8.33788,-8.15823,-0.666457,-0.347675,-0.486904;
        -3.82365,-21.1465,-7.96194,0.937887,-0.322435,0.412095
    ];

    % Rotation matrix for transforming six-dimensional force to flange frame
    f_TCP = rpyToRotationMatrix(pi,0,-pi/4);

    % Initialize pose and rotation matrix containers
    pose = zeros(6, 3);
    kdl_pose = six_poses(:, 4:6);
    
    % Transformation calculations
    for i = 1:6
        axis = kdl_pose(i, :)';
        norm_val = norm(axis);
        axis = axis / norm_val;
        
        % Create transformation matrix
        TCP_base = axang2rotm([axis' norm_val]);
        f_base = TCP_base * f_TCP;

        % Convert rotation matrix to roll, pitch, yaw
        [roll, pitch, yaw]= rotationMatrixToRPY(f_base);
        pose(i, :) = [roll, pitch, yaw];
    end

    % Perform load parameter identification
    LoadParameterIdentification(sixDForce, pose);
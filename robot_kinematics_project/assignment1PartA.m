% assignment1PartA.m
% MTRN4230 Assignment 1 24T2
% Name: JENG-YANG YU
% Zid:  z5446068

clear; clc;

host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
rtde = rtde(host, port);

disp("Enter the pickup position")
pickupJointConfiguration = readConfiguration();

clc;
disp("Move robot to dropoff position")
dropoffJointConfiguration = readConfiguration();

clc;
disp("Calculated pickup pose: ")
pickupPose = convertJointToPose(pickupJointConfiguration)
disp("Calculated dropoff pose: ")
dropoffPose = convertJointToPose(dropoffJointConfiguration)

disp("Set robot to remote control mode then click enter")
input('');


% RTDE says it's taking in [x,y,z,r,p,y] but 
% its actually taking in [x,y,z,(rotation vector)]
% 
% The below four lines converts students rpy pose, into one with a rotation
% vector 
Tp = rpy2tr(pickupPose(4:6));
pickupPose = [pickupPose(1:3), rotmat2vec3d(Tp(1:3, 1:3))]';

Td = rpy2tr(dropoffPose(4:6));
dropoffPose = [dropoffPose(1:3), rotmat2vec3d(Td(1:3, 1:3))]';


rtde.movel(pickupPose'+[0 0 20 0 0 0], 'pose')
rtde.movel(pickupPose', 'pose')
rtde.movel(pickupPose'+[0 0 20 0 0 0], 'pose')

rtde.movel(dropoffPose'+[0 0 20 0 0 0], 'pose')
rtde.movel(dropoffPose', 'pose')
rtde.movel(dropoffPose'+[0 0 20 0 0 0], 'pose')

% Function to convert user input to array
function configuration = readConfiguration()
    configuration = [];

    in = input('Enter joint configuration exactly in the form "j1,j2,j3,j4,j5,j6": ', 's');
    joints = split(in, ",");

    for joint = joints
        configuration = [configuration, str2double(joint)];
    end
end

% You must implement the following function
function outputPose = convertJointToPose(jointConfiguration)
    % Replace this with your implementation
    outputPose = jointConfiguration
    outputPose = deg2rad(outputPose)
    a1 = 0;
    a2 = -425;
    a3 = -392.2;
    a4 = 0;
    a5 = 0;
    a6 = 0;
    alpha1 = pi/2;
    alpha2 = 0;
    alpha3 = 0;
    alpha4 = pi/2;
    alpha5 = -pi/2;
    alpha6 = 0;
    d1 = 162.5;
    d2 = 0;
    d3 = 0;
    d4 = 133.3;
    d5 = 99.7;
    d6 = 99.6;
    T_01 = [cos(outputPose(1)) -sin(outputPose(1))*cos(alpha1) sin(outputPose(1))*sin(alpha1) a1*cos(outputPose(1));
           sin(outputPose(1)) cos(outputPose(1))*cos(alpha1) -cos(outputPose(1))*sin(alpha1) a1*sin(outputPose(1));
           0 sin(alpha1) cos(alpha1) d1;
           0 0 0 1]
    T_12 = [cos(outputPose(2)) -sin(outputPose(2))*cos(alpha2) sin(outputPose(2))*sin(alpha2) a2*cos(outputPose(2));
           sin(outputPose(2)) cos(outputPose(2))*cos(alpha2) -cos(outputPose(2))*sin(alpha2) a2*sin(outputPose(2));
           0 sin(alpha2) cos(alpha2) d2;
           0 0 0 1]
    T_23 = [cos(outputPose(3)) -sin(outputPose(3))*cos(alpha3) sin(outputPose(3))*sin(alpha3) a3*cos(outputPose(3));
           sin(outputPose(3)) cos(outputPose(3))*cos(alpha3) -cos(outputPose(3))*sin(alpha3) a3*sin(outputPose(3));
           0 sin(alpha3) cos(alpha3) d3;
           0 0 0 1]
    T_34 = [cos(outputPose(4)) -sin(outputPose(4))*cos(alpha4) sin(outputPose(4))*sin(alpha4) a4*cos(outputPose(4));
           sin(outputPose(4)) cos(outputPose(4))*cos(alpha4) -cos(outputPose(4))*sin(alpha4) a4*sin(outputPose(4));
           0 sin(alpha4) cos(alpha4) d4;
           0 0 0 1]
    T_45 = [cos(outputPose(5)) -sin(outputPose(5))*cos(alpha5) sin(outputPose(5))*sin(alpha5) a5*cos(outputPose(5));
           sin(outputPose(5)) cos(outputPose(5))*cos(alpha5) -cos(outputPose(5))*sin(alpha5) a5*sin(outputPose(5));
           0 sin(alpha5) cos(alpha5) d5;
           0 0 0 1]
    T_56 = [cos(outputPose(6)) -sin(outputPose(6))*cos(alpha6) sin(outputPose(6))*sin(alpha6) a6*cos(outputPose(6));
           sin(outputPose(6)) cos(outputPose(6))*cos(alpha6) -cos(outputPose(6))*sin(alpha6) a6*sin(outputPose(6));
           0 sin(alpha6) cos(alpha6) d6;
           0 0 0 1]
    T_06 = T_01*T_12*T_23*T_34*T_45*T_56;
    R = T_06(1:3,1:3);
    r = atan2(R(3, 2), R(3, 3));
    p = atan2(-R(3, 1), sqrt(R(3, 2)^2 + R(3, 3)^2));
    y = atan2(R(2, 1), R(1, 1));
    outputPose = [T_06(1:3,4)',r,p,y]
    % You must not use RTDE at all in this implementation (it
    % must be done from first principles)
end
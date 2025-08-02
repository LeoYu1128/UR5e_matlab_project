% calculateMaxLinearVelocity.m
% MTRN4230 Assignment 1 24T2
% Name: JENG-YANG YU
% Zid:  z5446068

%% Function you must complete
% You must implement the following function
function maxLinearVelocity = calculateMaxLinearVelocity(jointPositions,jointVelocities)
dh = [
    0   162.5   0       pi/2;
    0   0       -425       0;
    0   0       -392.2     0;
    0   133.3    0      pi/2;
    0   99.7     0     -pi/2;
    0   99.6     0         0;
];
UR5e = SerialLink(dh, 'name', 'UR5e');    
% Write your implementation here
maxLinearVelocity = 0;
% Initialize variables
v_max = 0;
% Assuming joints and jointVelocities are matrices with rows corresponding to time steps
for i = 1:size(jointPositions, 1)
    % Get current joint positions and velocities for this time step
    q = jointPositions(i, :);
    q_dot = jointVelocities(i, :);
    % Calculate Jacobian matrix at this joint configuration
    % Replace with your actual Jacobian calculation method
    J = UR5e.jacob0(q);
    % Calculate end-effector linear velocity: v = J * q_dot'
    v_end_effector = J * q_dot';
    % Calculate magnitude of linear velocity
    v_mag = norm(v_end_effector(1:3));  % Consider only linear velocity
    % Update maximum velocity if current velocity is larger
    if v_mag > v_max
       v_max = v_mag;
    end
end
    maxLinearVelocity = v_max;
end
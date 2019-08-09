clear
clc
close all
%%

des_pose = [0;0];
start_pose = [-3;-3];
end_pose = [3;3];
traj = [start_pose(1):0.1:end_pose(1); start_pose(2):0.1:end_pose(2)];

deadzone = 0.5;
vel_max = 1.5;
kp = 0.8;
x_0 = 1;
alpha = 0.4;
x_vel = zeros(1,length(traj));
y_vel = zeros(1,length(traj));
euclidean_distance = zeros(1,length(traj));
velocity_request = zeros(1,length(traj));
for i = 1:length(traj)
    x_error = (des_pose(1) - traj(1,i));
    y_error = (des_pose(2) - traj(2,i));
    
    euclidean_distance(i) = sqrt( x_error^2 + y_error^2);
    
    if euclidean_distance(i) < deadzone
        velocity_request(i) = 0;
    else
        velocity_request(i) = vel_max * 1/(1+kp * exp(-(euclidean_distance(i) - x_0)/alpha));
    end
    
    theta = atan2(y_error, x_error) - atan2(traj(2,i), traj(1,i));
    x_vel(i) = velocity_request(i) * cos(theta);
    y_vel(i) = velocity_request(i) * sin(theta);
end

figure()
plot(euclidean_distance , velocity_request, 'b','LineWidth',3)
title('REEF Control Lookup Control')
xlabel('Euclidean Distance m')
ylabel('Velocity Request m/s')
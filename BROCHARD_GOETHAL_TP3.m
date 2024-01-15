%% Context 

% Tracking a autonomous car is a fundamental task and challenge in several applications
% as well as in urban environments as in aeronautic for plane inspection. Indeed, classically,
% it is equipped with several quantities of sensor (radar, lidar, ultra sound) which we need to
% track its position in its environment at the same time that the car moves. To resolve the
% tracking problem, one of the solutions is to place ourselves in a statistical context (to take
% into account the uncertainties on the measurements) by using the Bayesian filter and more
% particularly its analytical form given by Kalman filter and its variants.
% The aim of this tutorial is to familiarize with conventional tracking algorithms to resolve this
% problem by implementing two extensions of the Kalman filter (KF): the extended KF and the
% iterated extended KF.


%% Part 1) Starting up

% We assume the existence of a robot moving in a indoor 2D environment and embedded
% with an ultra-sound sensor. At each discrete instant k, its dynamic is characterized by its
% position pk and its orientation θk.

%% Question 2)

K = 100; %each instant
delta_t = 1; %time difference between two instant
p0 = [0; 0]; %Position at t = 0
theta0 = 0.1; %Angle at t = 0
w = 0.01; %Angular welocity
v = [1; 1]; %Robot velocity
sigma_p = 0.01; %variance for the noise on the position
sigma_th = 10^-4; %variance for the noise on the angle

%Implementation

p_vect = zeros(2,K); %Vectors initialization
theta_vect = zeros(K,1);

p_vect(:,1) = p0; %Store initial values into the vectors
theta_vect(1) = theta0;

for k=2:K
    
    theta_k = theta_vect(k-1) + w*delta_t+randn(1)*sigma_th; %Angular equation
    theta_vect(k) = theta_k;

    R = Rot_mat(theta_vect(k-1));
    p_k = p_vect(:,k-1) + R*v*delta_t+sigma_p*randn(2,1); %Position equation
    p_vect(:,k) = p_k;

end

%% Plot

figure
plot(p_vect(1,:), p_vect(2,:))
xlabel("X-coordinates")
ylabel("Y-coordinates")
title("Trajectory of the car")
legend("Car's trajectory")

%% Question 4)

N = 3; % Initialization parameters
p1 = [40; 10];
p2 = [80; 25];
p3 = [120; 30];

p_i = [p1, p2, p3];

sigma_d = 0.1;

z_i_k = zeros(2*N,K);
d_i_k =  zeros(N,K);

for k=2:K

    theta_k = theta_vect(k-1) + w*delta_t+randn(1)*sigma_th; %Angular equation
    theta_vect(k) = theta_k;

    R = Rot_mat(theta_vect(k-1));
    p_k = p_vect(:,k-1) + R*v*delta_t+sigma_p*randn(2,1); %Position equation
    p_vect(:,k) = p_k;

    for i=1:N

        z_i_k(2*i-1:2*i,k) = R*p_i(:,i) + p_k;
        d_i_k(i,k) = norm(z_i_k(2*i-1:2*i,k)) + randn(1)*sigma_d;

    end

end

%% Plot

figure

subplot(1,1,1)
plot(p_vect(1,:), p_vect(2,:))
hold on
scatter(p_i(1,:), p_i(2,:))
xlabel("X-coordinates")
ylabel("Y-coordinates")
title("Trajectory of the car")
legend("Car's trajectory", "Position of each sensor")

%% Part 2) Implementation of tracking algorithms

%% Question 5)



%% Functions

function R = Rot_mat(theta)

    R = [cos(theta), sin(theta);
        -sin(theta), cos(theta)];

end
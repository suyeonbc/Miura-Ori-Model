% Miura N4B5 & 1 cell 
clc
clear all
close all

% Define origami
ori=generate_miuraori_structure(1);

% Define dimensions
ori.a = 0.1; %m
ori.beta = deg2rad(60);

% Origami Structure
ori.nodenum = 9; % node number
n = 3 * ori.nodenum; % Number of degrees of freedom

% Constraints
remove_index = [1 2 3 5 8 9 12 18 21 27]';
remove_index = sort(remove_index);
remove_index_plus_n = remove_index + n;
ori.remove_index_2 = unique([remove_index; remove_index_plus_n]);

%% Operation point Calculation
n_region = 12;
ori.n_region = n_region;
start_angle = 0;
end_angle = 120;
ori.theta0 = deg2rad(end_angle); % zero energy
interval_size = (end_angle-start_angle)/n_region;
ori.region_dangle = zeros(1, n_region);

ori.op_dangle=zeros(1,n_region);
ori.region_dangle=zeros(1,n_region);

for i = 1:n_region
    ori.op_dangle(i) = start_angle + (i - 0.5) * interval_size;
    ori.region_dangle(i) = start_angle + (i - 1) * interval_size;
end
ori.op_angle = deg2rad(ori.op_dangle);
op_angle = ori.op_angle;

% Operation point coordinates
ori.p_op = zeros(2*n, length(op_angle));  % 3 dimensions for points, number of angles
for i = 1:length(op_angle)
    ang = op_angle(i);
    ori.p_op(1:ori.nodenum*3, i) = ang2coordinate(ori,ang);  
    ori.p_op(:, i) = [ori.p_op(1:ori.nodenum*3, i); zeros(n, 1)];
    if i == 1
        ori = barlength(ori, ori.p_op(:, i));
    end
end

%% Define Crease and Facet Hinge
ori.k_crease = 0.01; % Nm/rad
ori.k_facet = 100;  % Nm/rad
ori.k_bar = 1.2*1e6; % N/m

% Mass matrix
m = 1*10^(-3); % kg
M_full = eye(n) * m;
ori.M = M_full;
ori.K_damp = eye(n) * (-0.12); % Damping matrix

% actuation 
ori.crease_actuated = [3]; % multiple crease possible [1 3]
ori.actuation_gain = ones(1,length(ori.crease_actuated));

% matrix building
n_region = numel(op_angle);
K = zeros(n, n, n_region);
A_full = zeros(2*n, 2*n, n_region);
C_full = zeros(2*n, 1, n_region);
B_full = zeros(2*n,length(ori.crease_actuated),n_region);

%% Build state space model for each operating points
for i = 1:n_region
    fprintf('%d번째 linearization\n', i)
    p=ori.p_op(:, i);
    radtheta = ori.op_angle(i);
    K(:,:,i) = K_global_build(p, radtheta, ori);
    A_full(:,:,i) = [zeros(n), eye(n); ori.M\K(:,:,i), ori.M\ori.K_damp];
    C_full(:,:,i) = C_build(p, i, ori);
    B_full(:,:,i) = crease_actuation_matrix(i,ori);
end

% Removing constraint indices & discretize
remove_idx = ori.remove_index_2;
num_remove_idx = length(remove_idx);
n_f = 2*n - num_remove_idx;

A= zeros(n_f, n_f, n_region);
B= zeros(n_f, length(ori.crease_actuated), n_region);

for i = 1:n_region
    A_t = A_full(:,:,i);
    B_t = B_full(:,:,i);
    A_t(remove_idx, :) = [];
    A_t(:, remove_idx) = [];
    B_t(remove_idx, :) = [];
    A(:,:,i) = A_t;
    B(:,:,i) = B_t;
end

C = C_full;
C(remove_idx, :) = [];

%% Continuous time to Discrete time
A_d = zeros(size(A, 1), size(A, 2), n_region);
B_d = zeros(size(B, 1), size(B, 2), n_region);
C_d = zeros(size(C, 1), n_region);

% Sampling period
T = 0.001;

for i = 1:n_region
    % Discretize
    A_d(:,:,i) = expm(A(:,:,i) * T);
    B_d(:,:,i) = integral(@(t) expm(A(:,:,i) * t) * B(:,:,i), 0, T, 'ArrayValued', true);
    C_d(:,i) = integral(@(t) expm(A(:,:,i) * t) * C(:,i), 0, T, 'ArrayValued', true);
end

ori.A_d = A_d;
ori.B_d = B_d;
ori.C_d = C_d;
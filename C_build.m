function C = C_build(x,n_op,ori)
%setting
node_bar = ori.node_bar;
node_crease = ori.node_crease;
node_facet =ori.node_facethinge;

theta = ori.op_angle(n_op);
theta0 = ori.theta0;
dtheta = (theta0-theta);
l12_ori = ori.barlength;
N=3*ori.nodenum;
M = ori.M;

p_full=x;

% Bar
f_bar = zeros(2*N,1);
f = zeros(N,1);
for i = 1:size(node_bar,1)
    node_ind_1 = node_bar(i, 1); %node i
    node_ind_2 = node_bar(i, 2); %node j

    ind_1_i = 3 * node_ind_1 - 2;
    ind_1_f = 3 * node_ind_1;
    ind_2_i = 3 * node_ind_2 - 2;
    ind_2_f = 3 * node_ind_2;

    % Calculate vector between nodes
    vec_12 = p_full(ind_2_i:ind_2_f) - p_full(ind_1_i:ind_1_f);

    % Calculate length and unit vector
    l12 = norm(vec_12); % current length
    e12 = vec_12' / l12;
    l12_0 = l12_ori(i);

    f1 = ori.k_bar*(l12-l12_0)/l12_0*e12;
    f2 = -ori.k_bar*(l12-l12_0)/l12_0*e12;

    f(ind_1_i:ind_1_f,1)=f(ind_1_i:ind_1_f)+f1'/M(ind_1_i:ind_1_i);
    f(ind_2_i:ind_2_f,1)=f(ind_2_i:ind_2_f)+f2'/M(ind_2_i:ind_2_i);
end
f_bar(N+1:2*N,1)=f;

% Hinge
f_hinge_crease = zeros(2*N,1);
f = zeros(N,1);
% Crease Hinge
for i = 1:length(node_crease(:,1))
    node_indices = node_crease(i, 1:4);
    ind_K_start = 3 * node_indices - 2;
    ind_K_end = 3 * node_indices;

    % Compute r vectors
    r31 = p_full(ind_K_start(3):ind_K_end(3)) - p_full(ind_K_start(1):ind_K_end(1));
    r41 = p_full(ind_K_start(4):ind_K_end(4)) - p_full(ind_K_start(1):ind_K_end(1));
    r32 = p_full(ind_K_start(3):ind_K_end(3)) - p_full(ind_K_start(2):ind_K_end(2));
    r31=r31';
    r41=r41';
    r32=r32';
    
    m = cross(r41,r31);
    n = cross(r31,r32);
    norm_m=norm(m);
    norm_n=norm(n);
    l13 = norm(r31);
    dot_product = dot(m, n);
    cos_theta = dot_product / (norm_m * norm_n);
    cos_theta = max(min(cos_theta, 1), -1);
    theta_rad = acos(cos_theta);
    %theta_deg = rad2deg(theta_rad);
    dtheta = pi-theta_rad;

    f4 = ori.k_crease*l13*dtheta*norm(r31)/norm(m)^2*m;
    f2 = ori.k_crease*l13*dtheta*-norm(r31)/norm(n)^2*n;
    f1 = ori.k_crease*l13*dtheta*((dot(r41,r31)/norm(r31)^2-1)*norm(r31)/norm(m)^2*m-dot(r32,r31)/norm(r31)^2*-norm(r31)/norm(n)^2*n);
    f3 = ori.k_crease*l13*dtheta*((dot(r32,r31)/norm(r31)^2-1)*-norm(r31)/norm(n)^2*n-dot(r41,r31)/norm(r31)^2*norm(r31)/norm(m)^2*m);

    f(ind_K_start(1):ind_K_end(1), 1) = ...
        f(ind_K_start(1):ind_K_end(1), 1) + f1' / M(ind_K_start(1), ind_K_start(1));
    f(ind_K_start(2):ind_K_end(2), 1) = ...
        f(ind_K_start(2):ind_K_end(2), 1) + f2' / M(ind_K_start(2), ind_K_start(2));
    f(ind_K_start(3):ind_K_end(3), 1) = ...
        f(ind_K_start(3):ind_K_end(3), 1) + f3' / M(ind_K_start(3), ind_K_start(3));
    f(ind_K_start(4):ind_K_end(4), 1) = ...
        f(ind_K_start(4):ind_K_end(4), 1) + f4' / M(ind_K_start(4), ind_K_start(4));
end

% Facet Hinge
for i = 1:length(node_facet(:,1))
    node_indices = node_facet(i, 1:4);
    ind_K_start = 3 * node_indices - 2;
    ind_K_end = 3 * node_indices;
    
    % Compute r vectors
    r31 = p_full(ind_K_start(3):ind_K_end(3)) - p_full(ind_K_start(1):ind_K_end(1));
    r41 = p_full(ind_K_start(4):ind_K_end(4)) - p_full(ind_K_start(1):ind_K_end(1));
    r32 = p_full(ind_K_start(3):ind_K_end(3)) - p_full(ind_K_start(2):ind_K_end(2));
    r31=r31';
    r41=r41';
    r32=r32';
    
    m = cross(r41,r31);
    n = cross(r31,r32);
    norm_m=norm(m);
    norm_n=norm(n);
    l13 = norm(r31);
    dot_product = dot(m, n);
    cos_theta = dot_product / (norm_m * norm_n);
    cos_theta = max(min(cos_theta, 1), -1);
    theta_rad = acos(cos_theta);
    %theta_deg = rad2deg(theta_rad);
    dtheta = pi-theta_rad;

    f4 = ori.k_facet*l13*dtheta*norm(r31)/norm(m)^2*m;
    f2 = ori.k_facet*l13*dtheta*-norm(r31)/norm(n)^2*n;
    f1 = ori.k_facet*l13*dtheta*((dot(r41,r31)/norm(r31)^2-1)*norm(r31)/norm(m)^2*m-dot(r32,r31)/norm(r31)^2*-norm(r31)/norm(n)^2*n);
    f3 = ori.k_facet*l13*dtheta*((dot(r32,r31)/norm(r31)^2-1)*-norm(r31)/norm(n)^2*n-dot(r41,r31)/norm(r31)^2*norm(r31)/norm(m)^2*m);

    f(ind_K_start(1):ind_K_end(1), 1) = ...
        f(ind_K_start(1):ind_K_end(1), 1) + f1' / M(ind_K_start(1), ind_K_start(1));
    f(ind_K_start(2):ind_K_end(2), 1) = ...
        f(ind_K_start(2):ind_K_end(2), 1) + f2' / M(ind_K_start(2), ind_K_start(2));
    f(ind_K_start(3):ind_K_end(3), 1) = ...
        f(ind_K_start(3):ind_K_end(3), 1) + f3' / M(ind_K_start(3), ind_K_start(3));
    f(ind_K_start(4):ind_K_end(4), 1) = ...
        f(ind_K_start(4):ind_K_end(4), 1) + f4' / M(ind_K_start(4), ind_K_start(4));
end


f_hinge_crease(N+1:2*N,1)=f;
C = f_hinge_crease+f_bar;
end
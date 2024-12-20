% Global stiffness matrix
function K = K_global_build(x, ori)

% Extract necessary parameters from ori
node_bar = ori.node_bar;
node_crease = ori.node_crease;
node_facet =ori.node_facethinge;

l12_ori = ori.barlength;
n=3*ori.nodenum;

% Initialize the global stiffness matrix
K_bar = zeros(n);
K_hinge_crease = zeros(n);
K_hinge_facet = zeros(n);

p_full = x;

% for bar element
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
    l12 = norm(vec_12); %current length
    e12 = vec_12' / l12;
    l12_0 = l12_ori(i);
    
    dfidxi = -e12.' * e12 + (-l12/l12_0 + 1)*eye(3);
    dfidxj = e12.' * e12 - (-l12/l12_0 + 1)*eye(3);

    % Calculate local stiffness matrix   
    k12_bar = ori.k_bar / l12 .* [dfidxi dfidxj; dfidxj dfidxi];    

    % Assemble global stiffness matrix
    K_bar(ind_1_i:ind_1_f, ind_1_i:ind_1_f) = K_bar(ind_1_i:ind_1_f, ind_1_i:ind_1_f) + k12_bar(1:3, 1:3);
    K_bar(ind_1_i:ind_1_f, ind_2_i:ind_2_f) = K_bar(ind_1_i:ind_1_f, ind_2_i:ind_2_f) + k12_bar(1:3, 4:6);
    K_bar(ind_2_i:ind_2_f, ind_1_i:ind_1_f) = K_bar(ind_2_i:ind_2_f, ind_1_i:ind_1_f) + k12_bar(4:6, 1:3);
    K_bar(ind_2_i:ind_2_f, ind_2_i:ind_2_f) = K_bar(ind_2_i:ind_2_f, ind_2_i:ind_2_f) + k12_bar(4:6, 4:6);
end

% for crease hinge element 
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
    % node 4 stiffness
    kj4 = (dot(r41,r31)/l13^2-1)*l13^2/norm_m^4*(m.'*m) + dot(r32,r31)/norm_m^2/norm_n^2*(n.'*m) + ...
        dtheta*(-m.'*r31/l13/norm_m^2+(m.'*cross(r31-r41,m)+cross(r31-r41,m).'*m)*l13/norm_m^4)'; 
    kl4 = -l13^2/norm_m^2/norm_n^2*(n.'*m); 
    kk4 = (dot(r32,r31)/l13^2-1)*-1*l13^2/norm_m^2/norm_n^2*(n.'*m) - dot(r41,r31)/norm_m^4*(m.'*m) + ...
        dtheta*(m.'*r31/l13/norm_m^2+(m.'*cross(r41,m)+cross(r41,m).'*m)*l13/norm_m^4)'; 
    ki4 = (l13*(m.'*m) - dtheta*(m.'*cross(r31,m)+cross(r31,m).'*m))*l13/norm_m^4; 
    k4 = ori.k_crease*l13*[kj4 kl4 kk4 ki4];

    % node 2 stiffness
    ki2 = -l13^2/norm_m^2/norm_n^2*(m.'*n);
    kj2 = -(dot(r41,r31)/l13^2-1)*l13^2/norm_m^2/norm_n^2*(m.'*n) - dot(r32,r31)/norm(n)^4*(n.'*n) + ...
        dtheta*(m.*r31/l13/norm_n^2+(m.'*cross(r41,m)+cross(r41,m).'*m)*l13/norm_m^4)';  %
    kl2 = l13^2/norm_n^4*(n.'*n)+dtheta*(n.'*cross(r31,n)+cross(r31,n).'*n)*l13/norm_n^4; 
    kk2 = (dot(r32,r31)/l13^2-1)*l13^2/norm_n^4*(n.'*n) + dot(r41,r31)/norm_m^2/norm_n^2*(m.'*n) + ...
        dtheta*(-n.'*r31/norm_n^2/l13-(n.'*cross(r31-r32,n)+cross(r31-r32,n).'*n)*l13/norm_n^4)'; 
    k2 = ori.k_crease*l13*[kj2 kl2 kk2 ki2];
    
    q = acos(dot(r41,r31)/norm(r41)/norm(r31));
    l1p_crease = norm(r41)*cos(q);
    l3p_crease = l13-l1p_crease;
    k1         = -(l3p_crease/l13*k4 + l3p_crease/l13*k2);
    k3         = -(l1p_crease/l13*k4 + l1p_crease/l13*k2);


    % Update stiffness matrices for k1 (1st node)
    for j = 1:4
        K_hinge_crease(ind_K_start(1):ind_K_end(1), ind_K_start(j):ind_K_end(j)) = ...
            K_hinge_crease(ind_K_start(1):ind_K_end(1), ind_K_start(j):ind_K_end(j)) + k1(:, (j-1)*3+1:j*3);
    end

    % Update stiffness matrices for k2 (2nd node)
    for j = 1:4
        K_hinge_crease(ind_K_start(2):ind_K_end(2), ind_K_start(j):ind_K_end(j)) = ...
            K_hinge_crease(ind_K_start(2):ind_K_end(2), ind_K_start(j):ind_K_end(j)) + k2(:, (j-1)*3+1:j*3);
    end

    % Update stiffness matrices for k3 (3rd node)
    for j = 1:4
        K_hinge_crease(ind_K_start(3):ind_K_end(3), ind_K_start(j):ind_K_end(j)) = ...
            K_hinge_crease(ind_K_start(3):ind_K_end(3), ind_K_start(j):ind_K_end(j)) + k3(:, (j-1)*3+1:j*3);
    end

    % Update stiffness matrices for k4 (4th node)
    for j = 1:4
        K_hinge_crease(ind_K_start(4):ind_K_end(4), ind_K_start(j):ind_K_end(j)) = ...
            K_hinge_crease(ind_K_start(4):ind_K_end(4), ind_K_start(j):ind_K_end(j)) + k4(:, (j-1)*3+1:j*3);
    end

    fprintf('%d th spring of hinge\n', i)

end

% for facet hinge element
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

    dtheta_fac = pi-theta_rad;

    % node 4 stiffness
    kj4 = (dot(r41,r31)/l13^2-1)*l13^2/norm_m^4*(m.'*m) + dot(r32,r31)/norm_m^2/norm_n^2*(n.'*m) + ...
        dtheta_fac*(-m.'*r31/l13/norm_m^2+(m.'*cross(r31-r41,m)+cross(r31-r41,m).'*m)*l13/norm_m^4)'; 
    kl4 = -l13^2/norm_m^2/norm_n^2*(n.'*m); 
    kk4 = (dot(r32,r31)/l13^2-1)*-1*l13^2/norm_m^2/norm_n^2*(n.'*m) - dot(r41,r31)/norm_m^4*(m.'*m) + ...
        dtheta_fac*(m.'*r31/l13/norm_m^2+(m.'*cross(r41,m)+cross(r41,m).'*m)*l13/norm_m^4)'; 
    ki4 = (l13*(m.'*m) - dtheta_fac*(m.'*cross(r31,m)+cross(r31,m).'*m))*l13/norm_m^4; 
    k4 = ori.k_facet*l13*[kj4 kl4 kk4 ki4];


    % node 2 stiffness
    ki2 = -l13^2/norm_m^2/norm_n^2*(m.'*n);  %
    kj2 = -(dot(r41,r31)/l13^2-1)*l13^2/norm_m^2/norm_n^2*(m.'*n) - dot(r32,r31)/norm_n^4*(n.'*n) + ...
        dtheta_fac*(m.*r31/l13/norm_n^2+(m.'*cross(r41,m)+cross(r41,m).'*m)*l13/norm_m^4)';  %
    kl2 = l13^2/norm_n^4*(n.'*n)+dtheta_fac*(n.'*cross(r31,n)+cross(r31,n).'*n)*l13/norm_n^4; 
    kk2 = (dot(r32,r31)/l13^2-1)*l13^2/norm_n^4*(n.'*n) + dot(r41,r31)/norm_m^2/norm_n^2*(m.'*n) + ...
        dtheta_fac*(-n.'*r31/norm_n^2/l13-(n.'*cross(r31-r32,n)+cross(r31-r32,n).'*n)*l13/norm_n^4)'; 
    k2 = ori.k_facet*l13*[kj2 kl2 kk2 ki2];
    
    q = acos(dot(r41,r31)/norm(r41)/norm(r31));
    l1p_crease = norm(r41)*cos(q);
    l3p_crease = l13-l1p_crease;
    k1         = -(l3p_crease/l13*k4 + l3p_crease/l13*k2);
    k3         = -(l1p_crease/l13*k4 + l1p_crease/l13*k2);


    % Update stiffness matrices for k1 (1st node)
    for j = 1:4
        K_hinge_facet(ind_K_start(1):ind_K_end(1), ind_K_start(j):ind_K_end(j)) = ...
            K_hinge_facet(ind_K_start(1):ind_K_end(1), ind_K_start(j):ind_K_end(j)) + k1(:, (j-1)*3+1:j*3);
    end

    % Update stiffness matrices for k2 (2nd node)
    for j = 1:4
        K_hinge_facet(ind_K_start(2):ind_K_end(2), ind_K_start(j):ind_K_end(j)) = ...
            K_hinge_facet(ind_K_start(2):ind_K_end(2), ind_K_start(j):ind_K_end(j)) + k2(:, (j-1)*3+1:j*3);
    end

    % Update stiffness matrices for k3 (3rd node)
    for j = 1:4
        K_hinge_facet(ind_K_start(3):ind_K_end(3), ind_K_start(j):ind_K_end(j)) = ...
            K_hinge_facet(ind_K_start(3):ind_K_end(3), ind_K_start(j):ind_K_end(j)) + k3(:, (j-1)*3+1:j*3);
    end

    % Update stiffness matrices for k4 (4th node)
    for j = 1:4
        K_hinge_facet(ind_K_start(4):ind_K_end(4), ind_K_start(j):ind_K_end(j)) = ...
            K_hinge_facet(ind_K_start(4):ind_K_end(4), ind_K_start(j):ind_K_end(j)) + k4(:, (j-1)*3+1:j*3);
    end

    fprintf('%dth spring of facet \n', i)
end

%K_bar
%K_hinge_crease
%K_hinge_facet

K = K_bar+K_hinge_crease+K_hinge_facet; % Return the global stiffness matrix
end

% compute_stiffness_matrix.m

function F = compute_force_matrix(ori, X)
    n = ori.n;
    F = zeros(n, 1);
    
    % Elastic Bars 
    for ibar = 1:size(ori.node_bar, 1)
        i = ori.node_bar(ibar, 1);
        j = ori.node_bar(ibar, 2);

        % Indices for DOFs
        idx_i = (i-1)*3 + (1:3);
        idx_j = (j-1)*3 + (1:3);

        % Node positions
        pi = X(idx_i, 1);
        pj = X(idx_j, 1);

        % Direction vector and length
        d = pj - pi;
        L = norm(d);
        L0 = ori.barlength(ibar);
        if L == 0
            continue; % Avoid division by zero
        end
        e = d / L; 

        % Spring constant
        k = ori.k_bar;
        fbar = k*(L-L0)/L0;

        % Assemble into global stiffness matrix
        F(idx_i) = F(idx_i) + fbar*e;
        F(idx_j) = F(idx_j) - fbar*e;
    end
    
    % Rotational Springs for Creases 
    for c = 1:size(ori.node_crease, 1)
        nodes = ori.node_crease(c, :);
        i = nodes(1);
        j = nodes(2);
        k = nodes(3);
        l = nodes(4);

        % Indices for DOFs
        idx_i = (i-1)*3 + (1:3);
        idx_j = (j-1)*3 + (1:3);
        idx_k = (k-1)*3 + (1:3);
        idx_l = (l-1)*3 + (1:3);

        r31 = - X(idx_k) + X(idx_i);
        r41 = - X(idx_l) + X(idx_i);
        r32 = - X(idx_k) + X(idx_j);
        m = cross(r41,r31);
        n = cross(r31,r32);
        norm_m=norm(m);
        norm_n=norm(n);
        
        cos_theta = dot(m,n) / (norm_m * norm_n);
        cos_theta = max(min(cos_theta, 1), -1);
        theta_rad = acos(cos_theta);
        theta_deg = rad2deg(theta_rad);
        
        fspring = ori.k_crease*(pi-theta_rad);
        fl = fspring.*m/norm_m;
        fj = - fspring.*n/norm_n;
        fi = - (fj*0.5+fl*0.5);
        fk = - (fj*0.5+fl*0.5);

        F(idx_i) = F(idx_i) - fi;
        F(idx_l) = F(idx_l) - fl;
        F(idx_j) = F(idx_j) - fj;
        F(idx_k) = F(idx_k) - fk;
    end
    
    % Rotational Springs for Facet Hinges
    for f = 1:size(ori.node_facethinge, 1)
        nodes = ori.node_facethinge(f, :);
        i = nodes(1);
        j = nodes(2);
        k = nodes(3);
        l = nodes(4);

        % Indices for DOFs
        idx_i = (i-1)*3 + (1:3);
        idx_j = (j-1)*3 + (1:3);
        idx_k = (k-1)*3 + (1:3);
        idx_l = (l-1)*3 + (1:3);

        r31 = - X(idx_k) + X(idx_i);
        r41 = - X(idx_l) + X(idx_i);
        r32 = - X(idx_k) + X(idx_j);
        m = cross(r41,r31);
        n = cross(r31,r32);
        norm_m=norm(m);
        norm_n=norm(n);
        
        cos_theta = dot(m,n) / (norm_m * norm_n);
        cos_theta = max(min(cos_theta, 1), -1);
        theta_rad = acos(cos_theta);
        theta_deg = rad2deg(theta_rad);
        
        fspring = ori.k_crease*(pi-theta_rad);
        fl = fspring.*m/norm_m;
        fj = - fspring.*n/norm_n;
        fi = - (fj*0.5+fl*0.5);
        fk = - (fj*0.5+fl*0.5);

        F(idx_i) = F(idx_i) - fi;
        F(idx_l) = F(idx_l) - fl;
        F(idx_j) = F(idx_j) - fj;
        F(idx_k) = F(idx_k) - fk;
    end
end

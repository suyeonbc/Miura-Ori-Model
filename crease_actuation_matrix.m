function B = crease_actuation_matrix(i,ori)
B = zeros(ori.nodenum*3*2,length(ori.crease_actuated));
crease_actuated = ori.crease_actuated;
actuation_gain = ori.actuation_gain;
node_position_initial=ori.p_op(:,i);
for i = 1:length(crease_actuated)
    force = zeros(ori.nodenum*3,1);
    % for hinge element (crease)
    hinge_number = crease_actuated(i);
    node_ind_1 = ori.node_crease(hinge_number,1);
    node_ind_2 = ori.node_crease(hinge_number,2);
    node_ind_3 = ori.node_crease(hinge_number,3);
    node_ind_4 = ori.node_crease(hinge_number,4);
    
    ind_1_start = 3*node_ind_1-2;
    ind_1_end   = 3*node_ind_1;
    ind_2_start = 3*node_ind_2-2;
    ind_2_end   = 3*node_ind_2;
    ind_3_start = 3*node_ind_3-2;
    ind_3_end   = 3*node_ind_3;
    ind_4_start = 3*node_ind_4-2;
    ind_4_end   = 3*node_ind_4;


    r31 = node_position_initial(ind_3_start:ind_3_end) - node_position_initial(ind_1_start:ind_1_end);
    r41 = node_position_initial(ind_4_start:ind_4_end) - node_position_initial(ind_1_start:ind_1_end);
    r32 = node_position_initial(ind_3_start:ind_3_end) - node_position_initial(ind_2_start:ind_2_end);
    m = cross(r41,r31);
    n = cross(r31,r32);

    ljk_crease = norm(r31);
    q          = acos(dot(r41,r31)/norm(r41)/norm(r31));
    d_crease   = norm(r41)*sin(q);

    f4 = m/norm(m) * actuation_gain(i) / d_crease;
    f2 = -n/norm(n)* actuation_gain(i) / d_crease;
    l1p_crease = norm(r41)*cos(q);
    l3p_crease = ljk_crease-l1p_crease;
    f1         = -(l3p_crease/ljk_crease*f4 + l3p_crease/ljk_crease*f2);   
    f3         = -(l1p_crease/ljk_crease*f4 + l1p_crease/ljk_crease*f2);
        
    force(ind_1_start:ind_1_end) = f1;
    force(ind_2_start:ind_2_end) = f2;
    force(ind_3_start:ind_3_end) = f3;
    force(ind_4_start:ind_4_end) = f4;
    B(ori.nodenum*3+1:end,i) = ori.M\force;
end
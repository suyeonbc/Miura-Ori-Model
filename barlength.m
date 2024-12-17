function ori = barlength(ori,x)
node_bar = ori.node_bar;

for i = 1:size(ori.node_bar,1)
    node_ind_1 = node_bar(i, 1);
    node_ind_2 = node_bar(i, 2);
    
    ind_1_i = 3 * node_ind_1 - 2;
    ind_1_f = 3 * node_ind_1;
    ind_2_i = 3 * node_ind_2 - 2;
    ind_2_f = 3 * node_ind_2;

    vec_12 = x(ind_1_i:ind_1_f) - x(ind_2_i:ind_2_f);
    ori.barlength(i,1) = norm(vec_12);
    norm(vec_12)
end

end
function ori = generate_miuraori_structure(n)
    % Generates a Miura-ori origami structure with 9 nodes
    % Input: n - Starting node number
    % Output: ori - Structure containing bar and node connectivity
    
    % Define nodes (total 9 nodes)
    nodes = n:(n + 8);
    center_node = nodes(5); % Center node
    
    % 1. Side Bars: Horizontal and vertical connections
    ori.node_bar_side = [nodes(1) nodes(2);
                         nodes(2) nodes(3);
                         nodes(7) nodes(8);
                         nodes(8) nodes(9);
                         nodes(1) nodes(4);
                         nodes(3) nodes(6);
                         nodes(4) nodes(7);
                         nodes(6) nodes(9)];
    
    % 2. Crease Bars: Connections to the center node
    ori.node_bar_crease = [nodes(2) center_node;
                           nodes(4) center_node;
                           center_node nodes(6);
                           center_node nodes(8)];
    
    % 3. Facet Bars: Diagonal connections
    ori.node_bar_facet = [nodes(2) nodes(4);
                          center_node nodes(3);
                          nodes(4) nodes(8);
                          center_node nodes(9)];
    
    % 4. Node Crease: Quadrilateral connections around the center node
    ori.node_crease = [center_node nodes(3) nodes(2) nodes(4);
                       nodes(4) nodes(8) center_node nodes(2);
                       nodes(6) nodes(9) center_node nodes(3);
                       nodes(8) nodes(9) center_node nodes(4)];
    
    % 5. Node Facet Hinge: Hinge connections for folding facets
    ori.node_facethinge = [nodes(2) center_node nodes(4) nodes(1);
                           nodes(3) nodes(6) center_node nodes(2);
                           nodes(8) nodes(7) nodes(4) center_node;
                           nodes(9) nodes(8) center_node nodes(6)];
    
    % 6. Combine all bars
    ori.node_bar = [ori.node_bar_crease;
                    ori.node_bar_side;
                    ori.node_bar_facet];
end

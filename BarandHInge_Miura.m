% Original Bar and Hinge Model
% Miura-Ori Origami 1 cell
% Trying to update to n*m cell

% Initial Setting
ori.a = 0.1; % m
ori.beta = deg2rad(60);

% Origami Structure
ori.nodenum = 9; % node number
n = 3 * ori.nodenum; % Number of degrees of freedom
ori.n = n;

% Define origami Structure
ori=generate_miuraori_structure(1);

% Constraints
remove_index = sort([1 2 3 5 8 9 12 18 21 27]');
remove_index_plus_n = remove_index + n;
ori.remove_index_2 = unique([remove_index; remove_index_plus_n]);
remove_index_2 = ori.remove_index_2;
active = true(n, 1);
active(remove_index) = false; % active mask


% Mass Matrix
m = 1*10^(-3); % kg
M_full = eye(n) * m;
ori.M_full = M_full;

% Initial configuration
initX = ang2coordinate(ori,deg2rad(10));
initV = zeros(size(initX));
initial_state_full = [initX; initV];
initX_reduced = initX(active);
initV_reduced = initV(active);
initial_state = [initX_reduced; initV_reduced]; % Flattened into a single vector

% Define external forces (no external forces initially)
F_ext = zeros(sum(active), 1);
% F_ext(3) = 10; 

% Define Crease and Facet Hinge Strengths
ori.k_crease = 0.01; % Nm/rad
ori.k_facet = 100;  % Nm/rad
ori.k_bar = 1.2*1e6; % N/m
ori = barlength(ori,initX);

% Damping
xi = 100; % damping coefficient
C_full = xi * M_full;
ori.C_full = C_full; % Store in ori for access in functions

%% Simulation Time
t_span = [0, 10]; 
t_steps = linspace(t_span(1), t_span(2), 5000);

% ODE solver execution
[ts, states] = ode45(@(t, state) bah_EOM_fun(t, state, ori, active, xi, F_ext, initial_state_full), t_steps, zeros(34,1), odeset('RelTol',1e-6,'AbsTol',1e-9));

% Extract results
displacements = states(:, 1:(end/2));
velocities = states(:, (end/2 + 1):end);

% Result Visualization
figure;
ax = axes('XLim',[-0.2 0.2],'YLim',[-0.1 0.4],'ZLim',[-0.1 0.4]);
xlabel('X'); ylabel('Y'); zlabel('Z');
view(70, 50);
title('Origami Structure Dynamics');
hold on;
grid on;

num_particles = ori.nodenum;
dim = 3;
quit = 5000;
current_disp = zeros(n,quit); 

for i = 1:1:quit %length(ts)
    i
    current_disp(active,i) = displacements(i, :);
    current_disp(:,i) = initX + current_disp(:,i);

    current_pos = reshape(current_disp(:,i), dim,num_particles)';
    cla(ax);
    
    % node plot
    scatter3(ax, current_pos(:,1), current_pos(:,2), current_pos(:,3), 50, 'filled', 'b');
    % bar plot
    for edge = 1:size(ori.node_bar, 1)
        node_i = ori.node_bar(edge, 1);
        node_j = ori.node_bar(edge, 2);
        plot3(ax, [current_pos(node_i,1), current_pos(node_j,1)], ...
                  [current_pos(node_i,2), current_pos(node_j,2)], ...
                  [current_pos(node_i,3), current_pos(node_j,3)], 'k-');
    end
    
    % zlim([-60 60]);

    drawnow;
    pause(0.01);

end

final_displacement = displacements(end, :);
final_positions = initX + reshape(final_displacement, num_particles, dim);
writematrix(final_positions, 'final_positions.txt', 'Delimiter', '\t');

disp('Run Complete!!!');

%% Save
save('1201_nonlinearworkspace.mat');

% Bar and Hinge EOM Function
function dstate_dt = bah_EOM_fun(t, state, ori, active, xi, F_ext,initial_state_full)
    % 상태 벡터 분리
    x_disp = state(1:(end/2));
    x_vel = state((end/2 + 1):end);
    t
    n = ori.n;
    % Reconstruct full state vectors by adding removed DOFs as zeros
    x_full_disp = initial_state_full(1:(end/2));
    x_full_vel = initial_state_full((end/2 + 1):end);
    %x_full_disp(active) = x_full_disp(active)+x_disp;
    temp = x_full_disp(active); 
    x_full_disp(active) = temp + x_disp;
    x_full_vel(active) = x_vel;
    
    % Compute stiffness matrix based on current positions
    Fk_full = compute_force_matrix(ori, x_full_disp);

    % Apply constraints by reducing matrices
    Fk_reduced = Fk_full(active);
    M_reduced = ori.M_full(active, active);
    C_reduced = xi * M_reduced;

    % External forces (already defined as F_ext)
    F_reduced = F_ext;

    % Compute accelerations
    x_acc = M_reduced \ (F_reduced - C_reduced * x_vel + Fk_reduced);

    % Combine derivatives
    dstate_dt = [x_vel; x_acc];
end

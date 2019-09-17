function coefficients = trajectory( t_init, t_fin, v_init, v_fin, j_pos_init, j_pos_fin )
    % qo = a0 + a1*t_init + a2*(t_init^2) + a3(t_init^3)
    % vo = a1 + 2*a2*t_init + 3*a3*(t_init^2)
    % qf = a0 + a1*t_final + a2*(t_final^2) + a3(t_final^3)
    % vf = a1 + 2*a2*t_final + 3*a3*(t_final^2)
    M = [1 t_init t_init^2 t_init^3;
        0 1 2*t_init 3*t_init^2;
        1 t_fin t_fin^2 t_fin^3;
        0 1 2*t_fin 3*t_fin^2];
    b = [j_pos_init;
        v_init;
        j_pos_fin;
        v_fin];
    coefficients = inv(M)*b;
end


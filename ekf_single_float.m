function [x_k_c, P_k, yaw_k_c] = ekf_single_float(x_k_1_c, P_k_1, z_k)
    % initialization
    persistent Q R;
    Q = [0.5, 0, 0.01;
         0, 0.5, 0.01;
         0.01, 0.01, 0.3];
    R = [0.5, 0, 0.1;
         0, 0.5, 0.1;
         0.1, 0.1, 0.6];

    % prediction
    x_k_p = attStateFcn(x_k_1_c);
    A_k = attStateJacobianFcn(x_k_1_c);
    P_k_p = A_k*P_k_1*A_k.' +  Q;

    % correction
    H_k = attMeasurementJacobianFcn(x_k_1_c);
    K_k = P_k_p*H_k.'/(H_k*P_k_p*H_k.' + R);
%     z_k = [1;1.2;0.90];
%     z_k = [VeloE(k); VeloN(k); compassDegToInvTanDeg(heading(k))/180*pi];
    z_k_p = attMeasurementFcn(x_k_p);
    x_k_c = x_k_p + K_k*(z_k - z_k_p);
    P_k = (eye(3) - K_k*H_k)*P_k_p;
    
    yaw_k_c = x_k_c(3);

end
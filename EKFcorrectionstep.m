function EKF_corr = EKFcorrectionstep(x_esti_new,P_cov_new,Jhk,Rk,zk)

    Kk = P_cov_new*Jhk'*pinv(Jhk*P_cov_new*Jhk'+Rk);
    P_cov_final = (eye(taille)-Kk*Hk)*P_cov_new;
    x_esti_final = x_esti_new + Kk*(zk-h(x_esti_new));
   

    EKF_corr = [x_esti_final, P_cov_final, Kk];

end

function h_val = h(xk)

    h_val = xk + Rot*delta_t;

end
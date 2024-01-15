function EKF_pred = EKFpredictionstep(x_esti,P_cov,Jf,Qk)

    x_esti_new = f(x_esti);
    P_cov_new = Jf*P_cov*Jf'+Qk;

    EKF_pred = [x_esti_new, P_cov_new];

end

function f_val = f(xk)

    f_val = xk + Rot*delta_t;

end


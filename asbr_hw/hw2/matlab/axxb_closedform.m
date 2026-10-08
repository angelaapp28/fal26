function X = axxb_closedform( e_bh, e_sc ) 
% e_bh: a 3x7 matrix that contain 3 forward kinematics measurements
%  obtained from tf_echo. The format of each row must be [tx ty tz qx qy qz qw] 

% e_sc: a 3x7 matrix that contain 3 AR tag measurements obtained from 
% tf_echo. The format of each row must be [tx ty tz qx qy qz qw]
% return: the 4x4 homogeneous transformation of the hand-eye calibration 
 
    E1 = quat2homo(e_bh(1, :));
    E2 = quat2homo(e_bh(2, :));
    E3 = quat2homo(e_bh(3, :));
    A1 = inv(E1) * E2;
    A2 = inv(E1) * E3;

    S1 = quat2homo(e_sc(1, :));
    S2 = quat2homo(e_sc(2, :));
    S3 = quat2homo(e_sc(3, :));
    B1 = S1 * inv(S2);
    B2 = S1 * inv(S3);
  
    a1 = get_alpha(A1(1:3, 1:3));

    a2 = get_alpha(A2(1:3, 1:3));

    b1 = get_beta(B1(1:3, 1:3));

    b2 = get_beta(B2(1:3, 1:3));

    fancy_A = [a1, a2, cross(a1, a2)];
    fancy_B = [b1, b2, cross(b1, b2)];

    Rx = fancy_A * inv(fancy_B);

    y_hash = [Rx * B1(1:3, 4) - A1(1:3, 4);
              Rx * B2(1:3, 4) - A2(1:3, 4)];

    A = [A1(1:3, 1:3) - eye(3);
         A2(1:3, 1:3) - eye(3)];

    tx = A \ y_hash;

    X = [Rx tx; 0 0 0 1];
    
end

function E = quat2homo(row) 
    t = row(1:3)';

    qx = row(4);
    qy = row(5);
    qz = row(6);
    qw = row(7);

    quat = [qw, qx, qy, qz];

    R = quat2rotm(quat);

    E = [R t; 0 0 0 1];
end

function alpha = get_alpha(rot)
    axang_rep = rotm2axang(rot);
    axis = axang_rep(1:3);
    theta = axang_rep(4);
    alpha = axis' * theta;
end

function beta = get_beta(rot)
    axang_rep = rotm2axang(rot);
    axis = axang_rep(1:3);
    theta = axang_rep(4);
    beta = axis' * theta; 
end

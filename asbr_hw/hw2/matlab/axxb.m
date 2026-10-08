function X=axxb( e_bh, e_sc )
% e_bh: a Nx7 matrix that contain N forward kinematics measurements 
% obtained from tf_echo. The format of each row must be [tx ty tz qx qy qz qw]
% e_sc: a Nx7 matrix that contain N AR tag measurements obtained from 
% tf_echo. The format of each row must be [tx ty tz qx qy qz qw]
% return: the 4x4 homogeneous transformation of the hand-eye calibration

    % first call Rx
    % need all the alphas and betas

    % next is tx 
    % need a 3x3xN matrix of RA and RB | 3xN tA and tB | and Rx

    N = size(e_bh, 1);
    
    alphas = zeros(3, N - 1);
    betas = zeros(3, N - 1);

    RA = zeros(3, 3, N - 1);
    RB = zeros(3, 3, N - 1);
    tA = zeros(3, N - 1);
    tB = zeros(3, N - 1);

    for i = 1:N - 1
        % e_bh
        bh_matrix1 = quat2homo(e_bh(i,:));
        bh_matrix2 = quat2homo(e_bh(i + 1, :));

        A_i = inv(bh_matrix1) * bh_matrix2;

        rotm_bh = A_i(1:3, 1:3);
        alphas(:, i) = get_alpha(rotm_bh);

        RA(:,:,i) = A_i(1:3,1:3);
        tA(:,i) = A_i(1:3, 4);


        % e_sc

        sc_matrix1 = quat2homo(e_sc(i,:));
        sc_matrix2 = quat2homo(e_sc(i + 1,:));

        B_i = sc_matrix1 * inv(sc_matrix2);

        rotm_sc = B_i(1:3, 1:3);
        betas(:, i) =  get_beta(rotm_sc);

        RB(:,:,i) = B_i(1:3, 1:3);
        tB(:,i) = B_i(1:3,4);

    end

    Rx = solveRx(alphas, betas);

    tx = solveTx(RA, tA, RB, tB, Rx);

    X = [Rx, tx; 0, 0, 0, 1];

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

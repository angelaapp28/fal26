function Rx=solveRx( alphas, betas )
 % alphas: A 3xN matrix representing the skew symmetric matrices. That 
 % is, alpha = [a_1 ... a_N] Each a_i is a 3x1 vector.
 % betas: A 3xN matrix representing the skew symmetric matrices. That
 % is, betas = [b_1 ... b_N] . Each b_i  is a 3x1 vector.
 % return: The least squares solution to the 3x3 matrix Rx.
   
    M = zeros(3,3);
    N = size(alphas, 2);

    for i = 1:N
        M = M + alphas(:, i) * betas(:, i)';
    end
    
    Rx = M * inv(sqrtm(M' * M));
    Rx = real(Rx);

    if det(Rx) < 0
        Rx(:,3) = -Rx(:,3);
    end

end

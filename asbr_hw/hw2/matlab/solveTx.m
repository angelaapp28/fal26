function tx=solveTx( RA, tA, RB, tB, RX )
 % RA: a 3x3xN matrix with all the rotations matrices R_Ai
 % tA: a 3xN matrix with all the translation vectors t_Ai
 % RB: a 3x3xN matrix with all the rotations matrices R_Bi
 % tB: a 3xN matrix with all the translation vectors t_Bi
 % RX: the 3x3 rotation matrix Rx
 % return: the 3x1 translation vector tx

    lhs = [];
    rhs = [];
    
    for i = 1:size(RA,3)
        lhs = [lhs; eye(3,3) - RA(:,:,i)];
        rhs = [rhs; tA(:, i) - RX * tB(:, i)];
    end

    tx = lhs \ rhs;
 
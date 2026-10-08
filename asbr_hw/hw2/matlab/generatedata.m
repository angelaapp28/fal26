% Generate synthetic data to test hand-eye calibration. 
% e_bh and e_sc are Nx7 matrices that represent N E_bh and N E_sc 
% transformations. Each row is of the form [ tx ty tz qx qy qz qw ]
% were tx, ty and tz denote a translation and qx, qy, qz, qw a 
% quaternion. 
% X is a randomly generated hand-eye transformation
function [e_bh, e_sc, X] = generatedata(N) 
    % Assume we know the transformation between the world and the 
    % checkerboard
    E_bc = [ eye(3) [ 1; 0; 0 ]; 0 0 0 1 ];
    % Create a random X for generating the data
    X = randSE3();
    e_bh = [];
    e_sc = [];

    for i=1:N
        % extract the rotational aspect of the matrix

        E_bh = randSE3();
        ex_rotation_bh = E_bh(1:3,1:3);
        ex_translation_bh = E_bh(1:3, 4);

        quat_bh = rotm2quat(ex_rotation_bh); % qw qx qy qz
        new_transf_bh = [transpose(ex_translation_bh) quat_bh(2:4) quat_bh(1)]; % tx ty tz qx qy qz qw
        e_bh = [e_bh; new_transf_bh];
        
        
        % e_sc = e_bc * inverse(e_bh) * inverse(x)

        E_sc = inv(X) * inv(E_bh) * E_bc;
        ex_rotation_sc = E_sc(1:3, 1:3);
        ex_translation_sc = E_sc(1:3, 4);

        quat_sc = rotm2quat(ex_rotation_sc);
        new_transf_sc = [transpose(ex_translation_sc) quat_sc(2:4) quat_sc(1)];
        e_sc = [e_sc; new_transf_sc];
    end
end
    
% Generate a random SE3 transformation
function Rt = randSE3()
    angle1 = 2*pi*rand();
    angle2 = 2*pi*rand();
    angle3 = 2*pi*rand();

    rotation1 = [cos(angle1) -sin(angle1) 0;
                 sin(angle1) cos(angle1) 0;
                 0 0 1];

    rotation2 = [cos(angle2) 0 sin(angle2);
                0 1 0;
                -sin(angle2) 0 cos(angle2)];

    rotation3 = [1 0 0;
                0 cos(angle3) -sin(angle3);
                0 sin(angle3) cos(angle3)];
    
    R = rotation1 * rotation2 * rotation3;

    % properties of rotation matrix must hold
    % rotation * rotation_transpose = I
    % det(rotation) = 1

    t = 3 * rand(3,1);
    
    Rt = [ R t; 0 0 0 1];

end

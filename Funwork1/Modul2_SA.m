
syms k1


A = [0 0 1 0;
    0 0 0 1;
    -k1 k1 -1 1;
    k1 -k1-0.25 1 -2];


B = [0;
    0;
    1;
    0];


Qz = [B A*B (A^2)*B (A^3)*B]



% Calculate the determinant of Qz symbolically
det_Qz = det(Qz);

% Solve for k1 when the determinant is equal to zero
solutions = solve(det_Qz == 0, k1)
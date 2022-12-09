function F = fundamentalEightPoint(p1,p2)
% fundamentalEightPoint  The 8-point algorithm for the estimation of the fundamental matrix F
%
% The eight-point algorithm for the fundamental matrix with a posteriori
% enforcement of the singularity constraint (det(F)=0).
% Does not include data normalization.
%
% Reference: "Multiple View Geometry" (Hartley & Zisserman 2000), Sect. 10.1 page 262.
%
% Input: point correspondences
%  - p1(3,N): homogeneous coordinates of 2-D points in image 1
%  - p2(3,N): homogeneous coordinates of 2-D points in image 2
%
% Output:
%  - F(3,3) : fundamental matrix

Q = zeros(width(p1),9);
for points = 1 : width(p1)
    kron_prod = kron(p1(:,points), p2(:,points));
    Q(points,:) = kron_prod';
end
[~,~,V] = svd(Q);
vec_F = V(:,end);
F = reshape(vec_F, 3, 3);

[A,B,C] = svd(F);
sigma_vec = [B(1,1), B(2,2), B(3,3)];
minimum = min(sigma_vec);
[i,j] = find(B == minimum);
B(i,j) = 0;
F = A * B * C';




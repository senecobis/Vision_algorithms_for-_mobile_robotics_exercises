function P = linearTriangulation(p1,p2,M1,M2)
% LINEARTRIANGULATION  Linear Triangulation
%
% Input:
%  - p1(3,N): homogeneous coordinates of points in image 1
%  - p2(3,N): homogeneous coordinates of points in image 2
%  - M1(3,4): projection matrix corresponding to first image
%  - M2(3,4): projection matrix corresponding to second image
%
% Output:
%  - P(4,N): homogeneous coordinates of 3-D points

P = zeros(4, width(p1));
for points = 1 : width(p1)
    A = [skew(p1(:,points)) * M1; skew(p2(:,points)) * M2];
    [~,~,V] = svd(A);
    % V = V_t'; % output V of svd is not trasposed
    solution = V(:, end);
    P(:,points) = [solution(1)/solution(4); solution(2)/solution(4); solution(3)/solution(4); 1];
end



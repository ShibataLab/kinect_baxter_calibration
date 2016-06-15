% absolute_orientation_svd.m: Program to compute the absolute orientation between two point sets
% Author: Nishanth Koganti
% Date: 2016/06/15

% Source: http://math.stackexchange.com/questions/745234/calculate-rotation-translation-matrix-to-match-measurement-points-to-nominal-poi

% TODO:
% 1) Implement in python

function [R, t, c, err, xout] = absolute_orientation(x, y, mode)

% check for the number of samples
assert(isequal(size(x),size(y)), 'inputs must have the same size');
assert(size(x,2) > size(x,1), 'insufficient number of points');
N = size(x,2);

% compute the mean
x_bar = mean(x,2);
y_bar = mean(y,2);

% center the points (eliminate the translation)
xTemp = x - repmat(x_bar,1,N);
yTemp = y - repmat(y_bar,1,N);

% compute the covariance
Sigma = yTemp*xTemp';

% compute the optimal rotation and account for reflections
[U,D,V] = svd(Sigma);
S = diag([1 1 sign(det(U)*det(V))]); % for reflections
R = U*S*V';

% scale factor
c = trace(D*S)./sum(sum(xTemp.^2,2));

if mode == 1
    c = 1;
end

% the translation
t = y_bar - c*R*x_bar;

%Compute the residual error
err =0;
xout = zeros(size(x));

for i=1:N
    xout(:,i) = c*R*x(:,i) + t;
    d = y(:,i) - (c*R*x(:,i) + t);
    err = err + norm(d);
end
err = err/N;

end

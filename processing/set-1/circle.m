
N = 20;
center = [0, 0];
noiseVariance = 0;
radiusx = 1;
radiusy = 1;
skewx = 4;
skewy = 0;

points = zeros(N, 3);

for i=1:N
    theta = randn()*2*pi;
    [x, y] = pol2cart(theta, 1);

    x = x + sqrt(noiseVariance)*randn();
    y = y + sqrt(noiseVariance)*randn();
    point = [x, y, 1] + sqrt(noiseVariance)*[randn(1, 2) 0];

    A = [radiusx   skewx    center(1);
         skewy     radiusy  center(2);
         0         0        1];

    point = A * point';

    points(i, :) = point';
end

points = sortrows(points)
[az, el, r] = cart2sph(points(:, 1), points(:, 2), points(:, 3));
aer = sortrows([az, el, r])

figure;
%hist(points(:,3))
plot(aer(1,:), aer(3,:));
axis square

%K = corrcoef(points(1, :), points(2, :))
%R = cov(points(1, :), points(2, :))

figure;
%close all;
plot(points(:, 1), points(:, 2), '.');
axis square;
xlim([-3 3]);
ylim([-3 3]);
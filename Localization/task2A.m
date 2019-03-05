%A
sigma_range = 0.5;
sigma_angle = 0.25;
range = randn(10000,1)*sigma_range+10;
angle = randn(10000,1)*sigma_angle+0;
x = range .* cos(angle);
y = range .* sin(angle);


%B&C
figure(1)
sigma0 = [sigma_range^2, 0; 0, sigma_angle^2];
mu = [10, 0];
K2 = 9;
draw_ellipse(mu, sigma0, K2, 'Npoints', 60, 'r');
hold on;
scatter(range,angle,'filled','g')
var0 = cov(range, angle);
draw_ellipse(mu, var0, K2, 'Npoints', 60, 'b');

figure(2)
var = cov(x,y)
%x = 10 + d(range)
%y = 10*d(angle)
%cov(x,y) = cov(d(range), d(angle)) = 0
%cov(x,x) = 0.5
%cov(y,y) = 100*0.25*0.25 = 6.25
mu = [10; 0];
sigma =[sigma_range^2, 0; 0, 100*sigma_angle*sigma_angle];
K2 = 9;
plt = draw_ellipse(mu, sigma, K2, 'Npoints', 60, 'r');
hold on;
scatter(x,y,'filled','g')
axis([0,15,-10,10])
draw_ellipse(mu, var, K2, 'Npoints', 60, 'b');

%D
data = [range, angle];
data2 = [x,y];
mean = [10,0];
mean2 = [10,0];
out = zeros(10000, 1);
out2 = zeros(10000, 1);
inv_sigma0 = inv(sigma0);
inv_sigma = inv(sigma);
for i = 1:10000
    out(i,1) = (data(i,:)-mean)*inv_sigma0*(data(i,:)-mean)';
    out2(i,1) = (data2(i,:)-mean2)*inv_sigma*(data2(i,:)-mean2)';
end
num1 = size(find(out<1))
num2 = size(find(out<4))
num3 = size(find(out<9))

num1_ = size(find(out2<1))
num2_ = size(find(out2<4))
num3_ = size(find(out2<9))

%E
%change the variance of angle lower would make the distribution more like a normal distribution 







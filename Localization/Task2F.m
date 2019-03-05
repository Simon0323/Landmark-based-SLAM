%A
sigma_range = 0.5;
sigma_angle = 0.25;
mu = [10,0];
corr = [0.1, 0.5, 0.9];
for i= 1:3
    sigma_cross = sigma_range*sigma_angle*corr(i);
    sigma = [sigma_range^2, sigma_cross; sigma_cross, sigma_angle^2];
    chol(sigma)
    R = mvnrnd(mu, sigma, 10000);
    range = R(:,1);
    angle = R(:,2);
    x = range .* cos(angle);
    y = range .* sin(angle);
    
    figure(i)
    subplot(1,2,1)
    scatter(range,angle,'filled','g');
    axis([6,14,-1.5,1.5])
    subplot(1,2,2)
    scatter(x,y,'filled','g');
    axis([2,14,-10,10])
end

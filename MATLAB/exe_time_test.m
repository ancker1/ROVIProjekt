ransac_time = importdata('ransacTime.txt');
ransac_opt_time = importdata('ransacOptimizedTime.txt');

% Convert data fro ms -> s
ransac_time = ransac_time ./ 1000;
ransac_opt_time = ransac_opt_time ./ 1000;

%% Check for normality
figure('Name','QQ plot of Ransac execution times')
qqplot(ransac_time)

figure('Name','QQ plot of time-optimized Ransac execution times')
qqplot(ransac_opt_time)

% The data looks normal distributed

%% Descriptive Statistics
ransac_time_mean = mean(ransac_time)
ransac_time_std_dev = std(ransac_time)

ransac_opt_time_mean = mean(ransac_opt_time)
ransac_opt_time_std_dev = std(ransac_opt_time)
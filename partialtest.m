##[p_velocity, stru_velocity] = polyfit(timestamp, velo_LSM_detrended, 10);
##velo_LSM_rebaseline = stru_velocity.yf;
##velo_LSM_redetrended = velo_LSM_detrended - velo_LSM_rebaseline;
##
##
##%%disp_LSM_detrended = integral(velo_LSM_redetrended, timestamp(1), timestamp(2000));
##[p_displacement, stru_displacement] = polyfit(timestamp, disp_LSM_detrended, 10);
##disp_LSM_rebaseline = stru_displacement.yf;
##disp_LSM_redetrended = disp_LSM_detrended - disp_LSM_rebaseline;


##
##
##velocity_LSM_trend = figure;
##disp_LSM_trend = figure;
##
##figure(velocity_LSM_trend);
##subplot(2, 1, 1); 
##plot(timestamp, velo_LSM_detrended, 'r');
##hold on;
##plot(timestamp, velo_LSM_rebaseline, 'b');
##title('rawdata and baseline');
##xlabel('time/s');
##ylabel('velocity');
##subplot(2, 1, 2);
##plot(timestamp, velo_LSM_redetrended);
##title('redetrended');
##xlabel('time/s');
##ylabel('velocity');
##
##figure(disp_LSM_trend);
##subplot(2, 1, 1);
##plot(timestamp, disp_LSM_detrended, 'r');
##hold on;
##plot(timestamp, disp_LSM_rebaseline, 'b');
##title('rawdata and baseline');
##xlabel('time/s');
##ylabel('displacement');
##subplot(2, 1, 2);
##plot(timestamp, disp_LSM_redetrended);
##title('redetrended');
##xlabel('time/s');
##ylabel('displacement');



[u, v] = dwt(velo_LSM_detrended, 'sym');

velo_WD_baseline = 
velo_WD_detrended = velocity - velo_WD_baseline;  
disp_WD_baseline = 
disp_WD_detrended = displacement - disp_WD_baseline;

velocity_WD_trend = figure;
disp_WD_trend = figure;

figure(velocity_WD_trend);
subplot(2, 1, 1); 
scatter(timestamp, velocity, 'r');
hold on;
plot(timestamp, velo_WD_baseline, 'b');
subplot(2, 1, 2);
plot(timestamp, velo_WD_detrended);

figure(disp_WD_trend);
subplot(2, 1, 1);
scatter(timestamp, displacement, 'r');
hold on;
plot(timestamp, disp_WD_baseline, 'b');
subplot(2, 1, 2);
plot(timestamp, disp_WD_detrended);



##velo_LSM_rebaseline = sgolayfilt(velo_LSM_detrended, 3, 251);
##velo_LSM_redetrended = velo_LSM_detrended - velo_LSM_rebaseline;
##
##disp_LSM_rebaseline = sgolayfilt(disp_LSM_detrended, 3, 251);
##disp_LSM_redetrended = disp_LSM_detrended - disp_LSM_rebaseline;
##
##
##
##
##velocity_LSM_trend = figure;
##disp_LSM_trend = figure;
##
##figure(velocity_LSM_trend);
##subplot(2, 1, 1); 
##plot(timestamp, velo_LSM_detrended, 'r');
##hold on;
##plot(timestamp, velo_LSM_rebaseline, 'b');
##title('rawdata and baseline');
##xlabel('time/s');
##ylabel('velocity');
##subplot(2, 1, 2);
##plot(timestamp, velo_LSM_redetrended);
##title('redetrended');
##xlabel('time/s');
##ylabel('velocity');
##
##figure(disp_LSM_trend);
##subplot(2, 1, 1);
##plot(timestamp, disp_LSM_detrended, 'r');
##hold on;
##plot(timestamp, disp_LSM_rebaseline, 'b');
##title('rawdata and baseline');
##xlabel('time/s');
##ylabel('displacement');
##subplot(2, 1, 2);
##plot(timestamp, disp_LSM_redetrended);
##title('redetrended');
##xlabel('time/s');
##ylabel('displacement');




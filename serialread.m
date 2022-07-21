
pkg load instrument-control
pkg load nan
##pkg load signal
pkg load optim


                                                                                %%%%
%%%%%%%%%%%                                                                     %%  Initialize Serial Port
                                                                                %%%%

if(exist("serial") != 3)
  disp("No Serial Support");
endif

s1 = serial("/dev/tty.usbmodem14201")
pause(1);

set(s1, 'baudrate', 115200);
set(s1, 'bytesize', 8);
set(s1, 'parity', 'n');
set(s1, 'stopbits', 1);
set(s1, 'timeout', 50);

% set(s1, 'requesttosend', 'on');
% set(s1, 'dataterminalready', 'off');

srl_flush(s1);

                                                                                %%%%
%%%%%%%%%%%                                                                     %%  Read Data 
                                                                                %%%%
g = 9.8;
spN = 200;                                                                     % sampling number
spP = 0.05;                                                                      % sampling period

accX = zeros(spN, 1); accY = zeros(spN, 1); accZ = zeros(spN, 1);
magX = zeros(spN, 1); magY = zeros(spN, 1); magZ = zeros(spN, 1);
gyroX = zeros(spN, 1); gyroY = zeros(spN, 1); gyroZ = zeros(spN, 1);

roll = zeros(spN, 1); pitch = zeros(spN, 1);  yaw = zeros(spN, 1);
velocity = zeros(spN, 1); displacement = zeros(spN, 1);
timestamp = zeros(spN, 1);
acc_vertical = zeros(spN, 1);


while( !(char(srl_read(s1, 1)) == 1 && char(srl_read(s1, 1)) == 9 && char(srl_read(s1, 1)) == 6) )
end

fprintf('Sampling...');

for cnt = 1 : spN
  data = srl_read(s1, 4);timestamp(cnt) = typecast(uint8(data), 'single') / 1000.0;
  data = srl_read(s1, 4);accX(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);accY(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);accZ(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);magX(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);magY(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);magZ(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);gyroX(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);gyroY(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);gyroZ(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);roll(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);pitch(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);yaw(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);yaw_dmp(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);pitch_dmp(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);roll_dmp(cnt) = typecast(uint8(data), 'single');
  
  
  data = srl_read(s1, 4);yaw_ma(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);pitch_ma(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);roll_ma(cnt) = typecast(uint8(data), 'single');  
  
  data = srl_read(s1, 4);yaw_none(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);pitch_none(cnt) = typecast(uint8(data), 'single');
  data = srl_read(s1, 4);roll_none(cnt) = typecast(uint8(data), 'single');
  %data = srl_read(s1, 4);velocity(cnt) = typecast(uint8(data), 'single');
  %data = srl_read(s1, 4);displacement(cnt) = typecast(uint8(data), 'single');
  
  acc_vertical(cnt) = (accZ(cnt) - 1) * g * cos(deg2rad(roll(cnt))) * cos(deg2rad(pitch(cnt))); 
##  if(cnt > 1)
##    interval = timestamp(cnt) - timestamp(cnt - 1);
##    velocity(cnt) = velocity(cnt - 1) + acc_vertical(cnt) * interval;
##    displacement(cnt) = displacement(cnt -1) + velocity(cnt) * interval;
##  end
end

fclose(s1);
clear s1;

fprintf('Sampling Completed...');
fprintf('Data processing...');

                                                                                %%%%
%%%%%%%%%%%                                                                     %%  Filtering

                                                                                %%%%
##
##acc_vertical = acc_vertical - mean(acc_vertical); 
## 
##lp = fir1(40, 0.2);
##hp = fir1(40, 0.02, 'high');
##wndw = 10;
##acc_vertical_lp = filter(lp, 1, acc_vertical);
##acc_vertical_lphp = filter(hp, 1, acc_vertical_lp);
##acc_vertical_lphpma = filter(ones(wndw, 1) / wndw, 1, acc_vertical_lphp);
##for cnt = 2 : spN
##  interval = timestamp(cnt) - timestamp(cnt - 1);
##  velocity(cnt) = velocity(cnt - 1) + (acc_vertical_lphpma(cnt) + acc_vertical_lphpma(cnt - 1))/2 * interval;
##  displacement(cnt) = displacement(cnt - 1) + (velocity(cnt) + velocity(cnt - 1))/2 * interval;
##end
##
##
##acc_filter = figure;
##figure(acc_filter);
##plot(timestamp, acc_vertical, 'r');
##hold on;
##plot(timestamp, acc_vertical_lphpma);
##
##title('highpass-lowpass-moving average');
##xlabel('time/s');
##ylabel('acceleration');    
##                                                                                %%%%
##%%%%%%%%%%%                                                                     %%  Plot
##                                                                                %%%%
##
##rawData = figure;                                                               
##attitude = figure;
##
##figure(rawData);   
##title('rawdata of MPU9250');                                                                             
##subplot(3, 3, 1); plot(timestamp, accX);  ylabel('accX');
##subplot(3, 3, 2); plot(timestamp, accY);  ylabel('accY');
##subplot(3, 3, 3); plot(timestamp, accZ);  ylabel('accZ');
##subplot(3, 3, 4); plot(timestamp, magX);  ylabel('magX');
##subplot(3, 3, 5); plot(timestamp, magY);  ylabel('magY');
##subplot(3, 3, 6); plot(timestamp, magZ);  ylabel('magZ');
##subplot(3, 3, 7); plot(timestamp, gyroX);  ylabel('gyroX');
##subplot(3, 3, 8); plot(timestamp, gyroY);  ylabel('gyroY');
##subplot(3, 3, 9); plot(timestamp, gyroZ);  ylabel('gyroZ');
##
##  
##
##figure(attitude);
##title('attitude by Madgwick Sensor Fusion Method');
##subplot(3, 1, 1); plot(timestamp, roll);  ylabel('roll');
##subplot(3, 1, 2); plot(timestamp, pitch);  ylabel('pitch');
##subplot(3, 1, 3); plot(timestamp, yaw);  ylabel('yaw');
##
##
##fprintf('accX: average value = %f, variance = %f \n ', mean(accX), var(accX));
##fprintf('accY: average value = %f, variance = %f \n ', mean(accY), var(accY));
##fprintf('accZ: average value = %f, variance = %f \n ', mean(accZ), var(accZ));
##fprintf('magX: average value = %f, variance = %f \n ', mean(magX), var(magX));
##fprintf('magY: average value = %f, variance = %f \n ', mean(magY), var(magY));
##fprintf('magZ: average value = %f, variance = %f \n ', mean(magZ), var(magZ));
##fprintf('gyroX: average value = %f, variance = %f \n ', mean(gyroX), var(gyroX));
##fprintf('gyroY: average value = %f, variance = %f \n ', mean(gyroY), var(gyroY));
##fprintf('gyroX: average value = %f, variance = %f \n ', mean(gyroZ), var(gyroZ));
##fprintf('pitch: average value = %f, variance = %f \n ', mean(pitch), var(pitch));
##fprintf('roll: average value = %f, variance = %f \n ', mean(roll), var(roll));
##fprintf('yaw: average value = %f, variance = %f \n ', mean(yaw), var(yaw));
##
##
##
##
##                                                                                %%%%
##%%%%%%%%%%%                                                                     %%  Trend Term Elimination
##                                                                                %%%%
##
##                                                                                % Least Square Method 
##
##f_velocity = @(a, x) a * x;
##f_displacement = @(b, x) b * x.^2;
##p_volocity = nonlin_curvefit(f_velocity, 0, timestamp, velocity);
##p_displacement = nonlin_curvefit(f_displacement, 0, timestamp, displacement);
##velo_LSM_baseline = p_volocity * timestamp;
##velo_LSM_detrended = velocity - velo_LSM_baseline;
##disp_LSM_baseline = p_displacement * timestamp.^2;
##disp_LSM_detrended = displacement - disp_LSM_baseline;
##
####[velo_LSM_redetrended, velo_LSM_rebaseline] = detrend(velo_LSM_detrended, 10);                                                                                                 
####[disp_LSM_redetrended, disp_LSM_rebaseline] = detrend(disp_LSM_detrended, 10);
##
##
##
##velocity_LSM_trend = figure;
##disp_LSM_trend = figure;
##
##figure(velocity_LSM_trend);
##subplot(2, 1, 1); 
##plot(timestamp, velocity, 'r');
##hold on;
##plot(timestamp, velo_LSM_baseline, 'b');
##title('rawdata and baseline');
##xlabel('time/s');
##ylabel('velocity');
##subplot(2, 1, 2);
##plot(timestamp, velo_LSM_detrended);
##title('detrended');
##xlabel('time/s');
##ylabel('velocity');
##
##figure(disp_LSM_trend);
##subplot(2, 1, 1);
##plot(timestamp, displacement, 'r');
##hold on;
##plot(timestamp, disp_LSM_baseline, 'b');
##title('rawdata and baseline');
##xlabel('time/s');
##ylabel('displacement');
##subplot(2, 1, 2);
##plot(timestamp, disp_LSM_detrended);
##title('detrended');
##xlabel('time/s');
##ylabel('displacement');
##
##
##                                                                                % Wavelet Decomposition
##Fs = 40;                                                                        % Sample Frequency
##Fc = 1;                                                                         % Cut-off Frequency
##w = 'sym8';                                                                     % Wavelet Type
##thr_met = 's';                                                                  % Threshold Type
##
####lev = ceil(log2(Fs/Fc));
####velo_WD_baseline = wden(velocity, 'heursure', thr_met, 'one', lev, w); 
####velo_WD_detrended = velocity - velo_WD_baseline;  
####disp_WD_baseline = wden(displacement, 'heursure', thr_met, 'one', lev, w);
####disp_WD_detrended = displacement - disp_WD_baseline;
####
####velocity_WD_trend = figure;
####disp_WD_trend = figure;
####
####figure(velocity_WD_trend);
####subplot(2, 1, 1); 
####scatter(timestamp, velocity, 'r');
####hold on;
####plot(timestamp, velo_WD_baseline, 'b');
####subplot(2, 1, 2);
####plot(timestamp, velo_WD_detrended);
####
####figure(disp_WD_trend);
####subplot(2, 1, 1);
####scatter(timestamp, displacement, 'r');
####hold on;
####plot(timestamp, disp_WD_baseline, 'b');
####subplot(2, 1, 2);
####plot(timestamp, disp_WD_detrended);
##
##fprintf('velocity: drift = %f in %f seconds \n', velocity(spN) - velocity(1), timestamp(spN) - timestamp(1));
##fprintf('displacement: drift = %f in %f seconds \n', displacement(spN) - displacement(1), timestamp(spN) - timestamp(1));
##
##                                                                                %%%%
##%%%%%%%%%%%                                                                     %%  Wave Analysis by FFT (Fast Fourier Transformation)
##                                                                                %%%%
##
##
##
##
##AccSpectrum = fft(acc_vertical_lphpma);
##P2 = abs(AccSpectrum / spN);
##P1 = P2(1 : spN / 2 + 1);
##P1(2 : end - 1) = 2 * P1(2 : end - 1); 
##
##f = (1 / spP) * (0 : (spN / 2)) / spN; 
##AccFFT = figure;
##figure(AccFFT);
##plot(f, P1);
##title('Single-Sided Amplitude Spectrum of Acceleration');
##xlabel('|f(Hz)|');
##ylabel('|P1(f)|');                                                                                     
##
##WaveSpectrum = fft(disp_LSM_detrended);
##P4 = abs(WaveSpectrum / spN);
##P3 = P4(1 : spN / 2 + 1);
##P3(2 : end - 1) = 2 * P3(2 : end - 1); 
##
##f = (1 / spP) * (0 : (spN / 2)) / spN; 
##WaveFFT = figure;
##figure(WaveFFT);
##plot(f, P3);
##title('Single-Sided Amplitude Spectrum of Wave');
##xlabel('|f(Hz)|');
##ylabel('|P1(f)|');                                                                            
##                                                                                

                                                               
                                                                                
                                                                                
                                                                                
                                                                                
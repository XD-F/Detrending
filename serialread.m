%{ 
 Copyright (C) 2022 Feng

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <https://www.gnu.org/licenses/>.

 -*- texinfo -*-
 @deftypefn {} {@var{retval} =} SerialRead (@var{input1}, @var{input2})

 @seealso{}
 @end deftypefn

 Author: Feng <Feng@2308-FENG>
 Created: 2022-07-22
%}


pkg load nan
pkg load signal
pkg load optim
%%pkg load instrument-control % Uncomment these two lines if a set of hardware is used for data collection
%%data = SerialRead()
           
%%  Data of Simulation
timestamp = 0:0.01:100
acc_vertical = sin(timestamp)

           
%%  Filtering                                                                          

acc_vertical = acc_vertical - mean(acc_vertical); 
 
lp = fir1(40, 0.2);
hp = fir1(40, 0.02, 'high');
wndw = 10;
acc_vertical_lp = filter(lp, 1, acc_vertical);
acc_vertical_lphp = filter(hp, 1, acc_vertical_lp);
acc_vertical_lphpma = filter(ones(wndw, 1) / wndw, 1, acc_vertical_lphp);
for cnt = 2 : spN
  interval = timestamp(cnt) - timestamp(cnt - 1);
  velocity(cnt) = velocity(cnt - 1) + (acc_vertical_lphpma(cnt) + acc_vertical_lphpma(cnt - 1))/2 * interval;
  displacement(cnt) = displacement(cnt - 1) + (velocity(cnt) + velocity(cnt - 1))/2 * interval;
end


acc_filter = figure;
figure(acc_filter);
plot(timestamp, acc_vertical, 'r');
hold on;
plot(timestamp, acc_vertical_lphpma);

title('highpass-lowpass-moving average');
xlabel('time/s');
ylabel('acceleration');    
                                                                                %%%%
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

                                                               
                                                                                
                                                                                
                                                                                
                                                                                

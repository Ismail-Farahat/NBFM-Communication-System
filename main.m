%%%%%%%%%%%%%%%%%%%%% FM COMMNUNICATION SYSTEM %%%%%%%%%%%%%%%%%%%%%%%%%%
clear,clc,close all
%%%%%%%%%%%%%%%%%  Audio Reading  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf("===============  Audio Reading  =================== \n");
% Read The Audio
file_name = input("Please, Enter The Audio Name : ",'s');
[y, Fs] = audioread(file_name);     % Fs: Sampling Frequency
Y = fftshift(fft(y));               % Fourier Transform (Frequency Domain)

% necessary values to plot
Ns = size(y,1);                     % Samples Number
t = linspace(0, Ns/Fs, Ns);         % Generate Time, end time = Ns/Fs
Pvec = linspace(-Fs/2,Fs/2,Ns);     % Frequency values vector on x-axis

% Plot the audio in Time Domain
plot(t,y);
xlabel('Time (S)');
ylabel('Magnitude');
title("Signal Waveform in Time Domain");

% Plot the audio in Frequency Domain
figure;
subplot(2,1,1);
plot(Pvec, abs(Y))                  % Plot the Magnatuide
xlabel('Frequency (Hz)');
ylabel('Magnitude Spectrum');
title("Magnatuide Spectrum in Frequency Domain");
subplot(2,1,2);
plot(Pvec, angle(Y))                % Plot the Phase
xlabel('Frequency (Hz)');
ylabel('Phase Spectrum');
title("Phase Spectrum in Frequency Domain");

% Play The Audio
fprintf("AUDIO PLAYING ON: Audio Reading \n");
sound(y,Fs);
pause(Ns/Fs) % pausing to have some time between playing audios
fprintf("AUDIO PLAYING OFF: Audio Reading \n");

%%%%%%%%%%%%%%%%%  LOW PASS Filter  %%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf("=================  LOW PASS FILTER  ==================== \n");
signal_time = y;
signal_frequency = Y;

% band width of the voice (fm)
low_filter_BW = 4000;      % fm

% number of samples that will be filtered from both sides of the signal
n_filter = floor((Ns/Fs) * (Fs/2 - low_filter_BW));    % 4000 Hz
% LOW PASS Filter
lowpass_filter = ones(Ns,1);
lowpass_filter([(1:n_filter) (end-n_filter+1:end)]) = 0;
% to filter the signal to pass only low frequencies (>4000 Hz)
filtered_signal_freq = lowpass_filter .* signal_frequency;
% signal in time domian after passing low pass filter
filtered_signal_time = real(ifft(ifftshift(filtered_signal_freq)));

% plot the filtered signal in time domain
figure;
plot(t, filtered_signal_time)      
xlabel('Time (S)');
ylabel('Magnitude');
title("Signal Waveform in Time Domain (After Filter)");

% plot the filtered signal in frequency domain
figure;
subplot(2,1,1);
plot(Pvec, abs(filtered_signal_freq))  % plot the frequency magnatuide
xlabel('Frequency (Hz)');
ylabel('Magnitude Spectrum');
title("Magnatuide Spectrum in Frequency Domain (After Filter)");
subplot(2,1,2);
plot(Pvec, angle(filtered_signal_freq))    % Plot the Phase
xlabel('Frequency (Hz)');
ylabel('Phase Spectrum');
title("Phase Spectrum in Frequency Domain (After Filter)");

% Play the audio after Filter
fprintf("AUDIO PLAYING ON: Signal After Low Filter \n");
sound(filtered_signal_time,Fs)
pause(Ns/Fs) % pausing to have some time between playing audios
fprintf("AUDIO PLAYING OFF: Signal After Low Filter \n");

%%%%%%%%%%%%%%%%%  NBFM Modulation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf("===============  NBFM Modulation  =================== \n");

Fc = 100000;          % Carrier Frequency
Fs_FM = 5 * Fc;       % Sampling Frequency for FM modluation signal
A = 1;                % Carrier Amplitude
Kf = 2*pi*0.01;       % Frequency Modulation Constant

% resampling the signal with the new sampling frequenccy
resampled_signal_time = resample(filtered_signal_time,Fs_FM,Fs);

% update Ns, t, Pvec to plot the signal in time and frequecy domains
Ns_FM = size(resampled_signal_time,1);      % Samples Number
Pvec_FM = linspace(-Fs_FM/2,Fs_FM/2,Ns_FM); % Generate Frequency Vector
t_FM = linspace(0, Ns_FM/Fs_FM, Ns_FM);     % Generate Time Vector

% NBFM Modulation in time domain
phi_t = Kf * cumsum(resampled_signal_time);
modulated_signal_time = A * cos(2*pi*Fc * t_FM' + phi_t);
% Modulation in frequency domain
modulated_signal_freq = fftshift(fft(modulated_signal_time));

% plot the modulated signal in time domain
figure;
plot(t_FM, modulated_signal_time)      
xlabel('Time (S)');
ylabel('Magnitude');
title("Signal Waveform in Time Domain (After Modulation)");

% plot the modulated signal in frequency domain
figure;
subplot(2,1,1);
plot(Pvec_FM, abs(modulated_signal_freq))
xlabel('Frequency (Hz)');
ylabel('Magnitude Spectrum');
title("Magnatuide Spectrum in Frequency Domain (After Modultion)");
subplot(2,1,2);
plot(Pvec_FM, angle(modulated_signal_freq))         % Plot the Phase
xlabel('Frequency (Hz)');
ylabel('Phase Spectrum');
title("Phase Spectrum in Frequency Domain (After Modultion)");

%%%%%%%%%%%%%%%%%  NBFM Demodulation  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fprintf("===============  NBFM Demodulation  ================= \n");

% Differentiator Stage
demodulated_signal_time_diff = [diff(modulated_signal_time); 1];
% Envolpe Detector
demodulated_signal_time_ED = abs(hilbert(demodulated_signal_time_diff));
% DC Block, AC mean = 0 so DC value is the mean of the signal.
DC_value = mean(demodulated_signal_time_ED);
demodulated_signal_time_dcblock = demodulated_signal_time_ED - DC_value;
% to get the original signal, we need to divide the signal by constants
demodulated_signal_time = demodulated_signal_time_dcblock / (A * Kf);
% to get the original signal in its orginal sampling frequecy
demodulated_signal_time = resample(demodulated_signal_time,Fs,Fs_FM);

% the demodulated signal in the frequency domain
demodulated_signal_freq = fftshift(fft(demodulated_signal_time));

% update Ns, t, Pvec
Ns = size(demodulated_signal_time,1);     % Samples Number
t = linspace(0, Ns/Fs, Ns);               % Generate Time Vector
Pvec = linspace(-Fs/2,Fs/2,Ns);           % Frequency values Vector

% plot the demodulated signal in time domain
figure;
plot(t, demodulated_signal_time)      
xlabel('Time (S)');
ylabel('Magnitude');
title("Signal Waveform in Time Domain (After Demodulation)");

% plot the demodulated signal in frequency domain
figure;
subplot(2,1,1);
plot(Pvec, abs(demodulated_signal_freq))  % plot the signal magnatuide
xlabel('Frequency (Hz)');
ylabel('Magnitude Spectrum');
title("Magnitude Spectrum in Frequency Domain (After Demodultion)");
subplot(2,1,2);
plot(Pvec, angle(demodulated_signal_freq)) % Plot the Phase
xlabel('Frequency (Hz)');
ylabel('Phase Spectrum');
title("Phase Spectrum in Frequency Domain (After Demodultion)");

% Play the audio after Demodulation
fprintf("AUDIO PLAYING ON: Signal After Demodulation \n");
sound(demodulated_signal_time,Fs)
pause(Ns/Fs) % pausing to have some time between playing audios
fprintf("AUDIO PLAYING OFF: Signal After Demodulation \n");

%%%%%%%%%%%%%%%%%%%%%%%%%%%  THE END  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
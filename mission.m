%% Assignment 1 - Part B, Section B3 (Combining Signals)
%  Do not change before line 40
%  You will need to have generated Data1B.mat from 
%  GenerateDataAssignment1B.m before working with this file.

%  Clearing and preparing the workspace
clear; clc; close all;

%  Load assignment data from Data1B.mat
load('Data1B.mat', 'fs', 'muxSignal');% 

%  The variable loaded are:
%     fs      Sampling frequency for Sections B3
%  muxSignal  Multiplexed music signals, for Section B3.

%==================================================================
%
% Names of variables you will need for marking.
% Refer to the assignment sheet for details.
% Names of the variables are important,
% e.g. 'a1' is considered a different variable to 'A1'.
% Make sure variables have been declared as they appear in the list below.
% ---------------------------------------------
% ===== Part 2 =====
%   Ts              Sampling period
%   t               time vector
%   MUX             Fourier transform of muxSignal
%   MUX_shift       Shifted and scaled Fourier Transform
%   f               Frequency vector
%   freqshift       Frequency shifts (vector)
%   MagSpec         amplitude of peaks (vector)
%   PhaseSpec       phases of peaks (vector)
%   xdm             output of FDMDemux (matrix)
%   XDM             Fourier transform of xdm (matrix)
%   B               Bandwidth
%   filteredoutput  Filtered Output Signal
%   decodedtext     The Decoded Text Output 
% ---------------------------------------------
%====Enter your code below this line================================

Ts = length(muxSignal) / fs;   %Finds total time
t = linspace(0, Ts ,length(muxSignal)+1);   %Sets up time matrix
t(end) = [];
figure   %Plots signal against time
plot(t, muxSignal)
xlabel('Time (s)'), ylabel('Magnitude')
title('Signal')

MUX = fft(muxSignal);   %Finds Fourier transform of signal
f = linspace(-fs/2,fs/2, length(MUX)+1);
f(end) = [];

MUX_plot = abs(MUX)/fs;   %Eliminates negative region
MUX_shift = abs(fftshift(MUX))/fs;   %Shifts zero f components to centre
figure   %Plots Fourier and shifted Fourier
subplot(2,1,1)
plot(f, MUX_plot, 'b');
xlabel('Frequency (Hz)'), ylabel('Magnitude')
title('MUX')
subplot(2,1,2)
plot(f, MUX_shift , 'b');
xlabel('Frequency (Hz)'), ylabel('Magnitude')
title('MUXshift')

[a,b] = findpeaks( abs(MUX)/fs, 'MinPeakHeight' , 0.49);   %Finds peaks

freqmag = zeros(1,5);
freqloc = zeros(1,5);

for g=1:5   %Eliminates negative region
    freqmag(g) = a(g+5);
    freqloc(g) = b(g+5);
end

MagSpec = freqmag;   %Magnitudes of peaks
PhaseSpec = angle(MUX(freqloc));   %Phase of peaks

freqshift = f(freqloc);   %frequencies

figure   %Plots Fourier transform with highlighted peaks
plot(f, abs(MUX)/fs,freqshift, freqmag, 'or')
xlabel('Frequency (Hz)'), ylabel('Magnitude')
title('MUX with Positive Peaks')

xdm = FDMDemux(muxSignal,t,MagSpec,freqshift,PhaseSpec);   %sets up signals for Fourier transform
% Modulation signals undergo Fourier Transform to find bandwidth of f shift
XDM(1,:)=fft(xdm(1,:));
XDM(2,:)=fft(xdm(2,:));
XDM(3,:)=fft(xdm(3,:));
XDM(4,:)=fft(xdm(4,:));
XDM(5,:)=fft(xdm(5,:));

figure   %Finding Bandwidth for each signal through visual justification
subplot(5,2,1)
plot(f,abs((XDM(1,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Signal 1')
subplot(5,2,2)
plot(f,abs((XDM(1,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Zoomed Signal 1'), xlim([-1000 1000]), ylim([0 0.1])
subplot(5,2,3)
plot(f,abs((XDM(2,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Signal 2')
subplot(5,2,4)
plot(f,abs((XDM(2,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Zoomed Signal 2'), xlim([-5000 5000]), ylim([0 0.1])
subplot(5,2,5)
plot(f,abs((XDM(3,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Signal 3')
subplot(5,2,6)
plot(f,abs((XDM(3,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Zoomed Signal 3'), xlim([-5000 5000]), ylim([0 0.1])
subplot(5,2,7)
plot(f,abs((XDM(4,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Signal 4')
subplot(5,2,8)
plot(f,abs((XDM(4,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Zoomed Signal 4'), xlim([-1000 1000]), ylim([0 0.1])
subplot(5,2,9)
plot(f,abs((XDM(5,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Signal 5')
subplot(5,2,10)
plot(f,abs((XDM(5,:)))/fs)
xlabel('F (Hz)'), ylabel('Magnitude'), title('Zoomed Signal 5'), xlim([-1000 1000]), ylim([0 0.1])

B = [500, 3800, 3800, 500, 500];   %Bandwidth for each signal respectively
FilterText=abs(f)<B(1);   %Text signal bandwidth
FilterSong=abs(f)<B(2);   %Song signal bandwidth
%Frequency Signal receives filtering
filtered(1,:)=XDM(1,:).*FilterText; 
filtered(2,:)=XDM(2,:).*FilterSong;
filtered(3,:)=XDM(3,:).*FilterSong;
filtered(4,:)=XDM(4,:).*FilterText;
filtered(5,:)=XDM(5,:).*FilterText;
%Filtered frequencies inverse Fourier for final individual signals + DC
%offset removed
filteredoutput(1,:)=ifft(ifftshift(filtered(1,:))) - mean(ifft(ifftshift(filtered(1,:))));
filteredoutput(2,:)=ifft(ifftshift(filtered(2,:))) - mean(ifft(ifftshift(filtered(2,:))));
filteredoutput(3,:)=ifft(ifftshift(filtered(3,:))) - mean(ifft(ifftshift(filtered(3,:))));
filteredoutput(4,:)=ifft(ifftshift(filtered(4,:))) - mean(ifft(ifftshift(filtered(4,:))));
filteredoutput(5,:)=ifft(ifftshift(filtered(5,:))) - mean(ifft(ifftshift(filtered(5,:))));
%Plays 2 sound signals at sample rate
%sound(filteredoutput(2,:),fs);
%sound(filteredoutput(3,:),fs);
%Decodes 3 text signals at sample rate
decodedtext1 = A1BTextdecode (filteredoutput(1,:),fs);
decodedtext2 = A1BTextdecode (filteredoutput(4,:),fs);
decodedtext3 = A1BTextdecode (filteredoutput(5,:),fs);
decodedtext = [decodedtext1 newline decodedtext3 newline decodedtext2];


%Big Gay

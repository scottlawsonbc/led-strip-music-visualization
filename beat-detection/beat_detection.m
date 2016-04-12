function beat_detection

% Clear all previous variables to free up microphone and COM port
clear all;

% This array maps 0-255 values to values that humans percieve as linear
% If x = 60, then y = LED(x) is the human-friendly version of x
LED = [hex2dec('00'), hex2dec('00'), hex2dec('00'), hex2dec('00'), hex2dec('00'), hex2dec('00'), hex2dec('00'), hex2dec('00'), hex2dec('00'), hex2dec('00'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'),...
hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('01'), hex2dec('02'), hex2dec('02'), hex2dec('02'), hex2dec('02'), hex2dec('02'), hex2dec('02'), hex2dec('02'),...
hex2dec('02'), hex2dec('02'), hex2dec('03'), hex2dec('03'), hex2dec('03'), hex2dec('03'), hex2dec('03'), hex2dec('03'), hex2dec('04'), hex2dec('04'), hex2dec('04'), hex2dec('04'), hex2dec('04'), hex2dec('05'), hex2dec('05'), hex2dec('05'),...
hex2dec('05'), hex2dec('06'), hex2dec('06'), hex2dec('06'), hex2dec('07'), hex2dec('07'), hex2dec('07'), hex2dec('08'), hex2dec('08'), hex2dec('08'), hex2dec('09'), hex2dec('09'), hex2dec('0A'), hex2dec('0A'), hex2dec('0B'), hex2dec('0B'),...
hex2dec('0C'), hex2dec('0C'), hex2dec('0D'), hex2dec('0D'), hex2dec('0E'), hex2dec('0F'), hex2dec('0F'), hex2dec('10'), hex2dec('11'), hex2dec('11'), hex2dec('12'), hex2dec('13'), hex2dec('14'), hex2dec('15'), hex2dec('16'), hex2dec('17'),...
hex2dec('18'), hex2dec('19'), hex2dec('1A'), hex2dec('1B'), hex2dec('1C'), hex2dec('1D'), hex2dec('1F'), hex2dec('20'), hex2dec('21'), hex2dec('23'), hex2dec('24'), hex2dec('26'), hex2dec('27'), hex2dec('29'), hex2dec('2B'), hex2dec('2C'),...
hex2dec('2E'), hex2dec('30'), hex2dec('32'), hex2dec('34'), hex2dec('36'), hex2dec('38'), hex2dec('3A'), hex2dec('3C'), hex2dec('3E'), hex2dec('40'), hex2dec('43'), hex2dec('45'), hex2dec('47'), hex2dec('4A'), hex2dec('4C'), hex2dec('4F'),...
hex2dec('51'), hex2dec('54'), hex2dec('57'), hex2dec('59'), hex2dec('5C'), hex2dec('5F'), hex2dec('62'), hex2dec('64'), hex2dec('67'), hex2dec('6A'), hex2dec('6D'), hex2dec('70'), hex2dec('73'), hex2dec('76'), hex2dec('79'), hex2dec('7C'),...
hex2dec('7F'), hex2dec('82'), hex2dec('85'), hex2dec('88'), hex2dec('8B'), hex2dec('8E'), hex2dec('91'), hex2dec('94'), hex2dec('97'), hex2dec('9A'), hex2dec('9C'), hex2dec('9F'), hex2dec('A2'), hex2dec('A5'), hex2dec('A7'), hex2dec('AA'),...
hex2dec('AD'), hex2dec('AF'), hex2dec('B2'), hex2dec('B4'), hex2dec('B7'), hex2dec('B9'), hex2dec('BB'), hex2dec('BE'), hex2dec('C0'), hex2dec('C2'), hex2dec('C4'), hex2dec('C6'), hex2dec('C8'), hex2dec('CA'), hex2dec('CC'), hex2dec('CE'),...
hex2dec('D0'), hex2dec('D2'), hex2dec('D3'), hex2dec('D5'), hex2dec('D7'), hex2dec('D8'), hex2dec('DA'), hex2dec('DB'), hex2dec('DD'), hex2dec('DE'), hex2dec('DF'), hex2dec('E1'), hex2dec('E2'), hex2dec('E3'), hex2dec('E4'), hex2dec('E5'),...
hex2dec('E6'), hex2dec('E7'), hex2dec('E8'), hex2dec('E9'), hex2dec('EA'), hex2dec('EB'), hex2dec('EC'), hex2dec('ED'), hex2dec('ED'), hex2dec('EE'), hex2dec('EF'), hex2dec('EF'), hex2dec('F0'), hex2dec('F1'), hex2dec('F1'), hex2dec('F2'),...
hex2dec('F2'), hex2dec('F3'), hex2dec('F3'), hex2dec('F4'), hex2dec('F4'), hex2dec('F5'), hex2dec('F5'), hex2dec('F6'), hex2dec('F6'), hex2dec('F6'), hex2dec('F7'), hex2dec('F7'), hex2dec('F7'), hex2dec('F8'), hex2dec('F8'), hex2dec('F8'),...
hex2dec('F9'), hex2dec('F9'), hex2dec('F9'), hex2dec('F9'), hex2dec('FA'), hex2dec('FA'), hex2dec('FA'), hex2dec('FA'), hex2dec('FA'), hex2dec('FB'), hex2dec('FB'), hex2dec('FB'), hex2dec('FB'), hex2dec('FB'), hex2dec('FB'), hex2dec('FC'),...
hex2dec('FC'), hex2dec('FC'), hex2dec('FC'), hex2dec('FC'), hex2dec('FC'), hex2dec('FC'), hex2dec('FC'), hex2dec('FC'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'),...
hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FD'), hex2dec('FE'), hex2dec('FE'), hex2dec('FE'), hex2dec('FE'), hex2dec('FE'), hex2dec('FE'), hex2dec('FE'), hex2dec('FF'), hex2dec('FF')];

    % Returns a beat time series for each of the specified bands
    % This function uses some code from this student project: www..clear.rice.edu/elec301/Projects01/beat_sync/beatalgo.html
    function y = beat_detect(audio, bands, max_frequency)
        a = filterbank(audio, bands, max_frequency);
        b = hwindow(a, window_length, bands, max_frequency);
        y = diffrect(b, length(bands));
    end

% User settings
com_port = 'COM4';                % COM port for Arduino communication
audio_file = 'C:/120-bpm.mp3';    % Song file to use (.mp3)
microphone_sample_rate = 44100;   % Sampling rate of the microphone in Hz
use_mic = false;                  % Use microphone input instead of a .mp3 song
use_arduino = false;              % Use Arduino to visualize beats
arduino_buffer_samples = 256;
gain = 700;

% DSP settings
samples_per_frame = 512*1024;        % The number of samples to process in each frame

global in;

% MATLAB sucks are releasing COM ports. This force deletes the previous
% instance, if it exists.
try
    delete(instrfind({'Port'},{com_port}))
catch
end

if use_arduino
    s = serial(com_port);
    set(s, 'BaudRate', 250000);
    set(s, 'OutputBufferSize', 2048*4);
    fopen(s);
end

if use_mic
    in = dsp.AudioRecorder('SamplesPerFrame', samples_per_frame, 'SampleRate', microphone_sample_rate);
else
    in = dsp.AudioFileReader(audio_file, 'SamplesPerFrame', samples_per_frame);
    Speaker = dsp.AudioPlayer('SampleRate', in.SampleRate); 
end;

% Note: To get the sample rate, use in.SampleRate because this is accurate
% regardless of whether a microphone input or audio file is used as source
max_frequency = in.SampleRate/2;
bands = [0 200 400 800 1600 max_frequency];

% Window length is the length of the hanning window is that convolved with the
% audio. I believe a larger window = more blurring. Current value isn't
% optimal, it's just a guess.
window_length = samples_per_frame/in.SampleRate/2;

% Downsample factor is the factor by which the input audio is downsampled.
% The input audio is downsampled into just the right number of samples to
% send to the arduino for display.
downsample_factor = floor((samples_per_frame/arduino_buffer_samples)/(samples_per_frame/in.SampleRate));
tic

while(~isDone(in))
    audio = step(in);
    a = beat_detect(audio, bands, max_frequency);
    a = a*gain;
    r = a(:, 1) + a(:, 2);
    g = a(:, 3) + a(:, 4);
    b = a(:, 5) + a(:, 6);
  
    r = downsample(r, downsample_factor);
    g = downsample(g, downsample_factor);
    b = downsample(b, downsample_factor);
    
    % Three iterations of moving average filter approximates gaussian blur
    % This is done to prevent excess flicker on the output
    % Iteration 1
    r = smooth(r, 3);
    g = smooth(g, 3);
    b = smooth(b, 3);
    % Iteration 2
    r = smooth(r, 3);
    g = smooth(g, 3);
    b = smooth(b, 3);
    % Iteration 3
    r = smooth(r, 3);
    g = smooth(g, 3);
    b = smooth(b, 3);
    
    % Convert values into integer not exceeding 255
    r = floor(min(r, 255));
    g = floor(min(g, 255));
    b = floor(min(b, 255));
    
    % Convert to brightness values that humans percieve as linearly
    % increasing in brightness
    r = LED(r+1);
    g = LED(g+1);
    b = LED(b+1);

%    clf; 
%     semilogy(r);
%     hold on;
%     semilogy(g);
%     semilogy(b);
%     hold off;
%     ylim([0, 255]);
%     set(gca, 'YLim', [0, 300]);
%     drawnow;

    % Transmit values to Arduino where they are stored locally in a buffer.
    % The Arduino reads the serial information in the following way:
    %   byte = next serial byte
    %   if byte == 'r': r_buffer.enqueue(serial.parseInt())
    %   if byte == 'g': g_buffer.enqueue(serial.parseInt())
    %   if byte == 'b': b_buffer.enqueue(serial.parseInt())
    % Example serial strings:
    %  r40g60b10 --> Red: 40, Green: 60, Blue: 10
    %  r10r11r12 --> The next three red values are 10, 11, 12
    if use_arduino
        red   = sprintf('r%d', r);
        green = sprintf('g%d', g);
        blue  = sprintf('b%d', b);
        fprintf(s, red,   'sync');
        fprintf(s, green, 'sync');
        fprintf(s, blue,  'sync');
    end
    
    % Play the sound bytes over the speakers
    if ~use_mic
        step(Speaker, audio);
    end 
  toc
  tic
end

try
    fclose(s);
    delete(s);
    clear s;
    pause(in.QueueDuration);
    release(Speaker);
catch
end

end
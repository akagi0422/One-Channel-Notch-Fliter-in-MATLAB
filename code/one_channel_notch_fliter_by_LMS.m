%% 自适应陷波器仿真

clc;
clear;
close all;

%% ——造接收信号—————————————————————

f1 = 50; % 想接收信号的频率
Fs = 200e3; % 采样率
ts = 0:1/Fs:1-1/Fs; % 生成时间序列，长度1s，间隔为fs
signal_in = cos(2*pi*f1*ts);

% 在信号前后分别添加10000个零，得到一个长度为30000的信号
% zeros(1,Fs)生成包含Fs个0的数组
% signal_in = [zeros(1,Fs) signal_in zeros(1,Fs)]把0和信号拼接到一起
signal_in = [zeros(1,Fs) signal_in  zeros(1,Fs)];

% 添加噪声
N = length(signal_in); % 获取当前输入信号的长度
t = (1:N)*1/Fs; % 长度为1到N，频率为Fs生成时间序列，不知道原理但别动！

SNR = -10; % 信噪比 -10 dB
noise = randn(size(signal_in));
noise = noise - mean(noise);
signal_power = norm(signal_in)^2 / length(signal_in);
noise_power = signal_power / (10^(SNR/10));
noise = noise * sqrt(noise_power);
d = signal_in + noise; % 实际输入的信号
 
%%  ——造参考信号—————————————————————

x1 = cos(2*pi*t*f1);                 %参考输入1
x2 = sin(2*pi*t*f1);                 %参考输入2

%%  ——Notch滤波—————————————————————

w1 = 0.1;
w2 = 0.1;                       %权矢量初值
e = zeros(1,N);
y = zeros(1,N);
u = 0.05; % u为自适应步长，它会影响自适应收敛速度、失调情况以及滤波器的通带带宽

for i = 1:N                       %LMS算法
    y(i) = w1*x1(i)+w2*x2(i);
    e(i) = d(i)-y(i); 
    w1 = w1+u*e(i)*x1(i); 
    w2 = w2+u*e(i)*x2(i); 
end

%%  ——作图时域——————————————————————

figure;
subplot(4,1,1);
plot(t,signal_in);
ylim([-6, 6]);
xlabel('时间/s')
ylabel('幅度');
title('原始信号');

subplot(4,1,2);
plot(t,d);
ylim([-6, 6]);
xlabel('时间/s')
ylabel('幅度');
title('加入噪声干扰后的接收信号');

subplot(4,1,3);
plot(t,e);
ylim([-6, 6]);
xlabel('时间/s')
ylabel('幅度');
title('自适应陷波器输出信号');

subplot(4,1,4); 
plot(t,y);
ylim([-6, 6]);
xlabel('时间/s')
ylabel('幅度');
title('自适应滤波器输出信号');
 
%%  ——作图频域——————————————————————

figure;
Nfft=100000;
f=(0:1/Nfft:(1-1/Nfft))*Fs;

subplot(4,1,1);
plot(f,abs(fft(hilbert(signal_in(Fs:2*Fs)),Nfft)));xlim([0 30e3]);%axis([0 1000 0 1000]);
xlabel('频率/Hz')
title('原始信号 频谱');

subplot(4,1,2);plot(f,abs(fft(hilbert(d(Fs:2*Fs)),Nfft)));xlim([0 30e3]);%grid on;%axis([0 1000 0 5000]);
xlabel('频率/Hz')
title('加入噪声干扰后的接收信号  频谱');

subplot(4,1,3);plot(f,abs(fft(hilbert(e(Fs:2*Fs)),Nfft)));xlim([0 30e3]);%grid on;%axis([0 1000 0 1000]);
xlabel('频率/Hz')
title('自适应陷波器输出信号 频谱');

subplot(4,1,4);plot(f,abs(fft(hilbert(y(Fs:2*Fs)),Nfft)));xlim([0 30e3]);%grid on;%axis([0 1000 0 1000]);
xlabel('频率/Hz')
title('自适应滤波器输出信号 频谱');
 

clear;
clc;

%1.
[n,wn]=buttord(300,500,3,20,'s');%计算滤波器阶数
[b a]=butter(n,wn,'low','s');
g=tf(b,a);
figure;
subplot(2,1,1);
impulse(g);         %画脉冲响应
xlim([0 0.1]);
title('1. 滤波器冲激响应曲线');
ylabel('冲激响应强度');
xlabel('t/s');
grid on;
subplot(2,1,2);
[h,w]=freqs(b,a,1000);
plot(w,20*log10(abs(h)));       %画滤波器幅频特性
axis([0 600 -30 8]);
title('1. 滤波器幅频特性');
ylabel('幅值/dB');
xlabel('模拟频率/(rad/s)');
grid on;

%2.
f=100;
n=50;
[n wn]=buttord(0.4*f,0.8*f,3,15,'s');%计算模拟滤波器阶数
[b a]=butter(n,wn,'low','s');
[b a]=impinvar(b,a,f);      %转为数字滤波器
figure;
subplot(2,1,1);
impz(b,a,50,f);
title('2. 滤波器脉冲响应');%画脉冲响应
xlabel('nT/s  T=1/f f=100Hz');
grid on;
subplot(2,1,2);
[h w]=freqz(b,a);
plot(w,20*log10(abs(h)));%画滤波器幅频特性
title('2. 滤波器幅频特性 横轴为数字角频率');
ylabel('幅值/dB');
xlabel('归一化数字角频率\Omega/rad');
xlim([0 pi]);
grid on;



%3 
f=100;
[n wn]=buttord(2*f*tan(0.4/2),2*f*tan(0.8/2),3,15,'s');%计算预畸变的模拟滤波器阶数
[b a]=butter(n,wn,'low','s');
[b a]=bilinear(b,a,f);%双线性变换
figure;
subplot(2,1,1);
impz(b,a,50,f); %计算冲激响应
title('3. 滤波器脉冲响应');
xlabel('nT/s  T=1/f f=100Hz');
grid on;
subplot(2,1,2);
[h w]=freqz(b,a);
plot(w,20*log10(abs(h)));   %画滤波器幅频特性
title('3. 滤波器幅频特性 横轴为数字角频率');
ylabel('幅值/dB');
xlabel('归一化数字角频率\Omega/rad');
xlim([0 pi]);
grid on;
%与冲激响应不变法比较
%双线性变换法
figure;
subplot(2,2,1);
impz(b,a,50,f); %计算冲激响应
title('3. 双线性变换得到的滤波器脉冲响应');
xlabel('nT/s  T=1/f f=100Hz');
grid on;
subplot(2,2,3);
[h w]=freqz(b,a);
plot(w,20*log10(abs(h)));   %画滤波器幅频特性
title('3. 双线性变换得到的滤波器幅频特性');
ylabel('幅值/dB');
xlabel('归一化数字角频率\Omega/rad');
xlim([0 pi]);
grid on;
%冲激响应不变法
f=100;
n=50;
[n wn]=buttord(0.4*f,0.8*f,3,15,'s');%计算模拟滤波器阶数
[b a]=butter(n,wn,'low','s');
[b a]=impinvar(b,a,f);      %转为数字滤波器
subplot(2,2,2);
impz(b,a,50,f);
title('3. 冲激响应得到的滤波器脉冲响应');%画脉冲响应
xlabel('nT/s  T=1/f f=100Hz');
grid on;
subplot(2,2,4);
[h w]=freqz(b,a);
plot(w,20*log10(abs(h)));%画滤波器幅频特性
title('3. 冲激响应得到的滤波器幅频特性');
ylabel('幅值/dB');
xlabel('归一化数字角频率\Omega/rad');
xlim([0 pi]);
grid on;








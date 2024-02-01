clear;
clc;
%1.三角波
figure (1)%创建画布
figure (2)%创建画布
fs=1000;
x=zeros(1,9);
for i=0:8       %获得三角波序列
    if i<=4
        x(i+1)=i+1;
    end
    if i>=5
        x(i+1)=9-i;
    end
end
n=0:1:8;    %画三角波序列
figure(1);
subplot(3,1,1);
stem(n,x);
title('1.(1) 三角波序列');
xlabel('n');
ylabel('x1[n]');
figure(2);
subplot(2,3,1);
stem(n,x);
title('1.(1) 三角波序列');
xlabel('n');
ylabel('x1[n]');
%频谱用模拟频率做横轴
N=10000;
xk=fft(x,N);    %计算10000点fft
o=0:fs/N:fs-fs/N;
figure(1);
subplot(3,1,2);
plot(o,abs(xk));        %画10000点fft幅度谱
xlabel('f/Hz (模拟频率)');
ylabel('10000点FFT频谱幅度');
title('1.(1) N=10000三角波幅度谱');
figure(1);
subplot(3,1,3);     %画10000点fft相位谱
plot(o,angle(xk));
xlabel('f/Hz (模拟频率)');
ylabel('\theta/rad');
title('1.(1) N=10000三角波相位谱');

%频谱用数字角频率做横轴
o=0:2*pi/N:2*pi-2*pi/N;
figure(2);
subplot(2,3,2);         %画10000点fft幅度谱(连续)
plot(o,abs(xk));
xlabel('\Omega/rad (数字角频率)');
ylabel('10000点FFT频谱幅度');
xlim([0 2*pi]);
title('1.(1) N=10000三角波幅度谱(连续)');
subplot(2,3,3);         %画10000点fft幅度谱(离散)
stem(o,abs(xk));
xlabel('\Omega/rad (数字角频率)');
ylabel('10000点FFT幅度频谱');
xlim([0 2*pi]);
title('1.(1) N=10000三角波幅度谱(离散)');
figure(2);
subplot(2,3,5);          %画10000点fft相位谱(连续)
plot(o,angle(xk));
xlabel('\Omega/rad (数字角频率)');
ylabel('\theta/rad');
xlim([0 2*pi]);
title('1.N=10000三角波相位谱(连续)');
figure(2);
subplot(2,3,6);          %画10000点fft相位谱(离散)
stem(o,angle(xk));
xlabel('\Omega/rad (数字角频率)');
ylabel('\theta/rad');
xlim([0 2*pi]);
title('1.N=10000三角波相位谱(离散)');

%1.方波
figure(3);%创建画布
figure(4);%创建画布

x=ones(1,7);
x=[x 0 0];  %创建方波序列
n=0:1:8;
figure(3);  %画方波序列
subplot(3,1,1);
stem(n,x);
title('1.(2)方波序列');
ylim([0,1.5]);
xlabel('n');
ylabel('x2[n]');

figure(4);  %画方波序列
subplot(2,3,1);
stem(n,x);
title('1.(2)方波序列');
ylim([0,1.5]);
xlabel('n');
ylabel('x2[n]');

N=10000;
xk=fft(x,N);            %计算10000点FFT
%横轴用模拟频率
o=0:fs/N:fs-fs/N;
figure(3);
subplot(3,1,2);
plot(o,abs(xk));        %画FFT幅度谱
xlabel('f/Hz (模拟频率)');
ylabel('10000点FFT频谱幅度');
title('1.(2) 方波幅度谱');
figure(3);
subplot(3,1,3);             %画FFT相位谱
plot(o,angle(xk));
xlabel('f/Hz (模拟频率)');
ylabel('\theta/rad');
title('1.(2) 方波相位谱');


%横轴用数字频率
o=0:2*pi/N:2*pi-2*pi/N;
figure(4);
subplot(2,3,2);
plot(o,abs(xk));        %画FFT幅度谱 (连续)
xlabel('\Omega/rad (数字角频率)');
ylabel('10000点FFT频谱幅度');
title('1.(2) 方波幅度谱(连续)');
xlim([0 2*pi]);
figure(4);
subplot(2,3,5);             %画FFT相位谱(连续)
plot(o,angle(xk));
xlabel('\Omega/rad (模拟频率)');
ylabel('\theta/rad');
title('1.(2) 方波相位谱(连续)');
xlim([0 2*pi]);
figure(4);
subplot(2,3,3);
stem(o,abs(xk));        %画FFT幅度谱 (离散)
xlabel('\Omega/rad (数字角频率)');
ylabel('10000点FFT频谱幅度');
title('1.(2) 方波幅度谱(离散)');
xlim([0 2*pi]);
figure(4);
subplot(2,3,6);             %画FFT相位谱(离散)
stem(o,angle(xk));
xlabel('\Omega/rad (模拟频率)');
ylabel('\theta/rad');
title('1.(2) 方波相位谱(离散)');
xlim([0 2*pi]);


%2(1)
N=20000;
fs=1000;
x_1=zeros(1,15);
x_2=zeros(1,15);        
for i=0:1:9         %计算x1[n]
    if i<=4
        x_1(i+1)=i+1;
    end
    if i>=5
        x_1(i+1)=10-i;
    end
end
for i=0:1:11         %计算x2[n]
    if i<=5
        x_2(i+1)=2^i;
    end
    if i>=6
        x_2(i+1)=-2^(i-6);
    end
end
g=conv(x_1,x_2);        %计算两序列线性卷积
gc=cconv(x_1,x_2,12);  %计算两序列长度为12的圆周卷积
gc1=cconv(x_1,x_2,21);  %计算两序列长度为21圆周卷积

x_1k=fft(x_1,N);
x_2k=fft(x_2,N);    %计算两序列FFT
x_k=x_1k.*x_2k;     %计算两序列FFT相乘
g_k=fft(g,N);       %计算线性卷积序列FFT
gc_k=fft(gc,N);     %计算长度为12的圆周卷积序列FFT
gc_1k=fft(gc1,N);   %计算长度为21的圆周卷积序列FFT
g_ifft=ifft(x_k);   %计算两序列FFT相乘后的IFFT序列

figure;         %创建图层
n=0:1:14;
subplot(2,3,1);     %绘制x1[n]序列
stem(n,x_1);
title("2.(1) x1[n]");
ylabel('x1[n]');
xlabel('n');

subplot(2,3,4);
stem(n,x_2);
title("2.(1) x2[n]");       %绘制x2[n]序列
ylabel('x2[n]');
xlabel('n');

subplot(2,3,2);
stem(0:length(g)-1,g);
title("2.(1) x1[n]线性卷积x2[n]");       %绘制线性卷积序列
ylabel('y[n]');
xlabel('n');

subplot(2,3,5);
stem(0:length(gc)-1,gc);
title("2.(1) x1[n]与x2[n]长度为12的圆周卷积");       %绘制两序列长度为12的圆周卷积
ylabel('y1[n]');
xlabel('n');

subplot(2,3,6);
stem(0:length(gc1)-1,gc1);
title("2.(1) x1[n]与x2[n]长度为21的圆周卷积");       %绘制两序列长度为21的圆周卷积
ylabel('y2[n]');
xlabel('n');

subplot(2,3,3);
stem(0:length(g)-1,g_ifft(1:length(g)));
title("2.(1) IFFT(FFT(x1[n]).FFT(x2[n])序列");       %绘制两序列FFT相乘后的IFFT序列
ylabel('y3[n]');
xlabel('n');

%频谱绘制
figure %创建画布

n=0:1:14;
subplot(3,3,1);     %绘制x1[n]序列
stem(n,x_1);
title("2.(1) x1[n]");
ylabel('x1[n]');
xlabel('n');

subplot(3,3,4);
stem(n,x_2);
title("2.(1) x2[n]");       %绘制x2[n]序列
ylabel('x2[n]');
xlabel('n');

subplot(3,3,2);
o=0:2*pi/N:2*pi-2*pi/N;
plot(o,abs(x_1k));
title("2.(1) FFT(x1[n])幅度谱");       %绘制FFT(x1[n])幅度谱
ylabel('|X1(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,5);
plot(o,abs(x_2k));
title("2.(1) FFT(x2[n])幅度谱");       %绘制FFT(x2[n])幅度谱
ylabel('|X2(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,8);
plot(o,abs(x_k));
title("2.(1) FFT(x1[n]).FFT(x2[n])幅度谱");       %绘制FFT(x1[n]).FFT(x2[n])幅度谱
ylabel('|X2(\Omega).X1(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,7);
plot(o,abs(g_k));
title("2.(1) FFT(x1[n]线性卷积x2[n])幅度谱");       %绘制FFT(x1[n]线性卷积x2[n])幅度谱
ylabel('|Y(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,3);
plot(o,abs(gc_k));
title("2.(1) FFT(x1[n]与x2[n]长度为12的圆周卷积)幅度谱");       %绘制FFT(x1[n]与x2[n]长度为12的圆周卷积)幅度谱
ylabel('|Y1(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,6);
plot(o,abs(gc_1k));
title("2.(1) FFT(x1[n]与x2[n]长度为21的圆周卷积)幅度谱");       %绘制FFT(x1[n]与x2[n]长度为21的圆周卷积)幅度谱
ylabel('|Y2(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

%(2)
N=20000;
x_3=zeros(1,15);
x_4=zeros(1,15);        
for i=0:1:11        %计算x3[n]
    if i<=6
        x_3(i+1)=0.8^i;
    end
    if i>=7
        x_3(i+1)=i-3;
    end
end
for i=0:1:12            %计算x4[n]
    if i<=4
        x_4(i+1)=i-1;
    end
    if i>=5
        x_4(i+1)=-0.6^(i-6);
    end
end
g=conv(x_3,x_4);        %计算两序列线性卷积
gc=cconv(x_3,x_4,15);  %计算两序列长度为15的圆周卷积
gc1=cconv(x_3,x_4,24);  %计算两序列长度为24圆周卷积

x_3k=fft(x_3,N);
x_4k=fft(x_4,N);    %计算两序列FFT
x_k=x_3k.*x_4k;     %计算两序列FFT相乘
g_k=fft(g,N);       %计算线性卷积序列FFT
gc_k=fft(gc,N);     %计算长度为15的圆周卷积序列FFT
gc_1k=fft(gc1,N);   %计算长度为24的圆周卷积序列FFT
g_ifft=ifft(x_k);   %计算两序列FFT相乘后的IFFT序列

figure;         %创建图层
n=0:1:14;
subplot(2,3,1);     %绘制x3[n]序列
stem(n,x_3);
title("2.(1) x3[n]");
ylabel('x3[n]');
xlabel('n');

subplot(2,3,4);
stem(n,x_4);
title("2.(1) x4[n]");       %绘制x4[n]序列
ylabel('x4[n]');
xlabel('n');

subplot(2,3,2);
stem(0:length(g)-1,g);
title("2.(1) x3[n]线性卷积x4[n]");       %绘制线性卷积序列
ylabel('y[n]');
xlabel('n');

subplot(2,3,5);
stem(0:length(gc)-1,gc);
title("2.(1) x3[n]与x4[n]长度为15的圆周卷积");       %绘制两序列长度为15的圆周卷积
ylabel('y1[n]');
xlabel('n');

subplot(2,3,6);
stem(0:length(gc1)-1,gc1);
title("2.(1) x3[n]与x4[n]长度为24的圆周卷积");       %绘制两序列长度为24的圆周卷积
ylabel('y2[n]');
xlabel('n');

subplot(2,3,3);
stem(0:length(g)-1,g_ifft(1:length(g)));
title("2.(1) IFFT(FFT(x3[n]).FFT(x4[n])序列");       %绘制两序列FFT相乘后的IFFT序列
ylabel('y3[n]');
xlabel('n');

%频谱绘制
figure %创建画布

n=0:1:14;
subplot(3,3,1);     %绘制x3[n]序列
stem(n,x_3);
title("2.(1) x3[n]");
ylabel('x3[n]');
xlabel('n');

subplot(3,3,4);
stem(n,x_4);
title("2.(1) x4[n]");       %绘制x4[n]序列
ylabel('x4[n]');
xlabel('n');

subplot(3,3,2);
o=0:2*pi/N:2*pi-2*pi/N;
plot(o,abs(x_3k));
title("2.(1) FFT(x3[n])幅度谱");       %绘制FFT(x3[n])幅度谱
ylabel('|X3(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,5);
plot(o,abs(x_4k));
title("2.(1) FFT(x4[n])幅度谱");       %绘制FFT(x4[n])幅度谱
ylabel('|X4(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,8);
plot(o,abs(x_k));
title("2.(1) FFT(x3[n]).FFT(x4[n])幅度谱");       %绘制FFT(x3[n]).FFT(x4[n])幅度谱
ylabel('|X4(\Omega).X3(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,7);
plot(o,abs(g_k));
title("2.(1) FFT(x3[n]线性卷积x4[n])幅度谱");       %绘制FFT(x3[n]线性卷积x4[n])幅度谱
ylabel('|Y(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,3);
plot(o,abs(gc_k));
title("2.(1) FFT(x3[n]与x4[n]长度为15的圆周卷积)幅度谱");       %绘制FFT(x3[n]与x4[n]长度为15的圆周卷积)幅度谱
ylabel('|Y1(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);

subplot(3,3,6);
plot(o,abs(gc_1k));
title("2.(1) FFT(x3[n]与x4[n]长度为24的圆周卷积)幅度谱");       %绘制FFT(x3[n]与x4[n]长度为24的圆周卷积)幅度谱
ylabel('|Y2(\Omega)|');
xlabel('\Omega/rad');
xlim([0 2*pi]);


%3.(1)
f1=5;
f2=15;
f3=40;
x1=zeros(1,128);
x2=zeros(1,128);
x3=zeros(1,128);
f_1=5;
f_2=9;
for i=0:1:127       %不同频率采样
    x1(i+1)=sin(2*pi*f_1*i/f1)+sin(2*pi*f_2*i/f1);
     x2(i+1)=sin(2*pi*f_1*i/f2)+sin(2*pi*f_2*i/f2);
      x3(i+1)=sin(2*pi*f_1*i/f3)+sin(2*pi*f_2*i/f3);
end

x1_k=fft(x1,128);
x2_k=fft(x2,128);
x3_k=fft(x3,128);   %计算128点FFT

figure;     %创建画布
%画时域采样序列
subplot(3,3,1);  %画x1(n)
N=0:1:127;
stem(N,x1);
title('3.(1) f1=5Hz128点采样序列');
xlabel('n');
ylabel('x1(n)')

subplot(3,3,4);      %画x2(n)
stem(N,x2);
title('3.(1) f2=15Hz128点采样序列');
xlabel('n');
ylabel('x2(n)')


subplot(3,3,7);      %画x3(n)
stem(N,x3);
title('3.(1) f3=40Hz128点采样序列');
xlabel('n');
ylabel('x3(n)')
%画FFT频谱 (连续)
o=0:2*pi/128:2*pi-2*pi/128;     %画x1(n)的FFT幅度谱
subplot(3,3,2);
plot(o,abs(x1_k));
xlabel('\Omega/rad');
ylabel('|X1(\Omega)|');
xlim([0 2*pi]);
title('3.(1) f1=5Hz128点FFT幅度谱');

subplot(3,3,5);          %画x2(n)的FFT幅度谱
plot(o,abs(x2_k));
xlabel('\Omega/rad');
ylabel('|X2(\Omega)|');
xlim([0 2*pi]);
title('3.(1) f2=15Hz128点FFT幅度谱');

subplot(3,3,8);          %画x3(n)的FFT幅度谱
plot(o,abs(x3_k));
xlabel('\Omega/rad');
ylabel('|X3(\Omega)|');
xlim([0 2*pi]);
title('3.(1) f3=40Hz128点FFT幅度谱');

%画FFT频谱 (离散)
subplot(3,3,3);          %画x1(n)的FFT幅度谱
stem(o,abs(x1_k));
xlabel('\Omega/rad');
ylabel('|X1(\Omega)|');
xlim([0 2*pi]);
title('3.(1) f1=5Hz128点FFT幅度谱序列');

subplot(3,3,6);          %画x2(n)的FFT幅度谱
stem(o,abs(x2_k));
xlabel('\Omega/rad');
ylabel('|X2(\Omega)|');
xlim([0 2*pi]);
title('3.(1) f2=15Hz128点FFT幅度谱序列');

subplot(3,3,9);          %画x3(n)的FFT幅度谱
stem(o,abs(x3_k));
xlabel('\Omega/rad');
ylabel('|X3(\Omega)|');
xlim([0 2*pi]);
title('3.(1) f3=40Hz128点FFT幅度谱序列');

%3.(2)
figure;%创建画布
f=60;
x=zeros(1,64);
for i=0:1:63            %采样
    x(i+1)=sin(2*pi*f_1*i/f)+sin(2*pi*f_2*i/f);
end
N=0:1:63;
subplot(2,3,1);
stem(N,x);
xlabel('n');
ylabel('x1(n)');
title('3.(2) f=60Hz 64点采样序列');
x_=zeros(1,64);
y=[x x_];           %补零  
N=0:1:127;
subplot(2,3,4);
stem(N,y);
title('3.(2) f=60Hz 64点采样后补零64点的128点序列');
xlabel('n');
ylabel('x2(n)');
%计算FFT
xk=fft(x,64);
yk=fft(y,128);
%绘制64点采样序列的fft幅度谱
subplot(2,3,2);
o=0:2*pi/64:2*pi-2*pi/64;
plot(o,abs(xk));
xlabel('\Omega/rad');
ylabel('|X1(\Omega)|');
xlim([0 2*pi]);
title('3.(2) f=60Hz 64点采样序列的FFT幅度谱');

subplot(2,3,3);
o=0:2*pi/64:2*pi-2*pi/64;
stem(o,abs(xk));
xlabel('\Omega/rad');
ylabel('|X1(\Omega)|');
xlim([0 2*pi]);
title('3.(2) f=60Hz 64点采样序列的FFT幅度谱序列');
%绘制64点采样后补零64点的128点序列的fft幅度谱
subplot(2,3,5);
o=0:2*pi/128:2*pi-2*pi/128;
plot(o,abs(yk));
xlabel('\Omega/rad');
ylabel('|X2(\Omega)|');
xlim([0 2*pi]);
title('3.(2) f=60Hz 64点采样后补零64点的128点序列的FFT幅度谱');

subplot(2,3,6);
o=0:2*pi/128:2*pi-2*pi/128;
stem(o,abs(yk));
xlabel('\Omega/rad');
ylabel('|X2(\Omega)|');
xlim([0 2*pi]);
title('3.(2) f=60Hz 64点采样后补零64点的128点序列的FFT幅度谱序列');


%3.(3)
figure;%创建画布
f=60;
x=zeros(1,128);
for i=0:1:127
    x(i+1)=sin(2*pi*f_1*i/f)+sin(2*pi*f_2*i/f);
end
N=0:1:127;
subplot(2,3,1);     %画时域采样图
stem(N,x);
xlabel('n');
ylabel('x1(n)');
title('3.(3) f=60Hz 128点采样序列');

subplot(2,3,4);     %画时域采样图
stem(N,y);
xlabel('n');
ylabel('x2(n)');
title('3.(3) f=60Hz 64点采样后补零64点的128点序列');
%画FFT频谱
xk=fft(x,128);
subplot(2,3,2);
o=0:2*pi/128:2*pi-2*pi/128;
plot(o,abs(xk));
xlabel('\Omega/rad');
ylabel('|X1(\Omega)|');
xlim([0 2*pi]);
title('3.(3) f=60Hz 128点采样序列的FFT幅度谱');

subplot(2,3,3);
stem(o,abs(xk));
xlabel('\Omega/rad');
ylabel('|X1(\Omega)|');
xlim([0 2*pi]);
title('3.(3) f=60Hz 128点采样序列的FFT幅度谱序列');
%绘制64点采样后补零64点的128点序列的fft幅度谱
subplot(2,3,5);
plot(o,abs(yk));
xlabel('\Omega/rad');
ylabel('|X2(\Omega)|');
xlim([0 2*pi]);
title('3.(3) f=60Hz 64点采样后补零64点的128点序列的FFT幅度谱');

subplot(2,3,6);
stem(o,abs(yk));
xlabel('\Omega/rad');
ylabel('|X2(\Omega)|');
xlim([0 2*pi]);
title('3.(3) f=60Hz 64点采样后补零64点的128点序列的FFT幅度谱序列');


%4
[y,Fs]=audioread('laohu.wav');%读取音频信息
yk=fft(y);%做FFT
o=0:Fs/length(y):Fs-Fs/length(y);
t=1/Fs:1/Fs:length(y)/Fs;
figure;
subplot(4,2,1);
plot(t,y);
title('原信号波形图');
xlabel('t/s');
subplot(4,2,2);
plot(o,abs(yk));
xlabel('f/Hz');
title('原信号FFT波形图');

%合成信号
fs=8000;
T=8;
f1=261.63;
f2=293.66;
f3=329.63;
f4=349.23;
f5=392;
f=[f1 f2 f3 f4 f5];%所需频率数组
a=1;

y=0;%合成信号
y1=0;%合成信号加包络
y3=0;%合成信号加谐波
k=0.7;
t1=0:1/fs:0.5;
t2=0:1/fs:1;
so=[1 2 3 1 1 2 3 1 3 4 5 3 4 5 ];%乐谱
lt=[0.5 0.5 0.5 0.5  0.5 0.5 0.5 0.5  0.5 0.5  1  0.5 0.5  1];%节拍持续时间
for i=1:1:14%合成信号
    if lt(i)==0.5
        y=[y a*sin(2*pi*f(so(i))*t1)];
        y1=[y1 a*sin(2*pi*f(so(i))*t1).*exp(-4*t1)];%加指数包络
        y3=[y3 (0.5+1/16)*a*sin(2*pi*f(so(i))*t1)+0.25*a*sin(2*pi*2*f(so(i))*t1)+0.125*a*sin(2*pi*3*f(so(i))*t1)+0.0625*a*sin(2*pi*4*f(so(i))*t1)];
    end
    if lt(i)==1
        y=[y a*sin(2*pi*f(so(i))*t2)];
        y1=[y1 a*sin(2*pi*f(so(i))*t2).*exp(-4*t2)];%加指数包络
        y3=[y3 (0.5+1/16)*a*sin(2*pi*f(so(i))*t2)+0.25*a*sin(2*pi*2*f(so(i))*t2)+0.125*a*sin(2*pi*3*f(so(i))*t2)+0.0625*a*sin(2*pi*4*f(so(i))*t2)];
    end
    
    
end
%输出音频文件
audiowrite("5-3.wav",y,fs);
audiowrite("5-4.wav",y1,fs);
audiowrite("5-5.wav",y3,fs);
%画合成信号
t=1/fs:1/fs:length(y)/fs;
subplot(4,2,3);
plot(t,y);
xlabel('t/s');
title('合成信号波形图');
yk=fft(y);
o=0:fs/length(y):fs-fs/length(y);
subplot(4,2,4);
plot(o,abs(yk));
xlabel('f/Hz');
title('合成信号FFT波形图');
%画合成信号加入包络后的信号
subplot(4,2,5);
plot(t,y1);
xlabel('t/s');
title('合成信号加包络线波形图');
yk=fft(y1);
o=0:fs/length(y1):fs-fs/length(y1);
subplot(4,2,6);
plot(o,abs(yk));
title('f/Hz');
title('合成信号加包络后的FFT波形图');
%画合成信号加入谐波后的信号
subplot(4,2,7);
plot(t,y3);
xlabel('t/s');
title('合成信号加入二，三，四次谐波后的波形图');
yk=fft(y3);
o=0:fs/length(y3):fs-fs/length(y3);
subplot(4,2,8);
plot(o,abs(yk));
title('f/Hz');
title('合成信号加入二，三，四次谐波后的fft波形图');
%播放合成信号加入二，三，四次谐波后的音频
sound(y3);
pause(9);
%播放合成信号加入包络线后的音频
sound(y1);
pause(9);
%播放合成信号音频
sound(y);








 



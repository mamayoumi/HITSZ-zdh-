clear;
clc;
%1
A=444.128;
a=50*sqrt(2)*pi;
w1=a; %基本参数设置

f=[1000 300 200];
N=floor(0.064.*f); %频率即采样点设置
figure (1);%创建画布
figure (2);%创建画布
%计算不同采样频率对应的采样点和频谱
for i=1:1:3

x=zeros(1,N(i));
k=1;
for t=1/f(i):1/f(i):0.064 %u(t)在t等于0没有定义 ，从1/fs开始采样
x(k)=A*exp(-a*t)*sin(w1*t);
k=k+1;
end
n=0:1:N(i)-1;
figure(1);
subplot(3,3,3*i-2);
stem(n,x); %绘图设置
str=['1. ' num2str(f(i)) 'Hz 采样点'];
title(str);
xlabel('采样点n');
ylabel('x(n)');

figure(2);
subplot(3,2,2*i-1);
stem(n,x); %绘图设置
str=['1. ' num2str(f(i)) 'Hz 采样点'];
title(str);
xlabel('采样点n');
ylabel('x(n)');

%用模拟频率做横坐标
o=0:0.01:2*f(i);
fou=0;
re=0;
im=0;
for n=1:1:N(i)
re=re+x(n)*cos(2*pi*o*n/f(i));
im=im-x(n)*sin(2*pi*o*n/f(i));
end
fou=sqrt(re.^2+im.^2);
figure(1);
subplot(3,3,3*i-1);
plot(o,fou);
str=['1. ' num2str(f(i)) 'Hz 采样频谱(模拟频率)'];
title(str);
xlabel('f/Hz (模拟频率)');
ylabel('采样序列幅度谱');

%用数字角频率做横坐标
o=0:0.01:2*pi; %计算DTFT频谱，但将数字角频率取连续的值
fou=0;
re=0;
im=0;
for n=1:1:N(i) %计算DTFT频谱的实部和虚部
re=re+x(n)*cos(o*n);
im=im-x(n)*sin(o*n);
end
fou=sqrt(re.^2+im.^2);

%画由DTFT得到的频谱及绘图设置
figure(1);
subplot(3,3,3*i);
plot(o,fou);
xlabel('\Omega/rad (数字角频率0-2*pi)');
ylabel('采样序列幅度谱|X(\Omega)|');
str=['1. ' num2str(f(i)) 'Hz 采样频谱(数字角频率)'];
title(str);
xlim([0,2*pi]);

%用数字角频率做横坐标
o=0:2*pi/N(i):2*pi-2*pi/N(i); %计算DFT频谱，但将数字角频率取连续的值
fou=0;
re=0;
im=0;
for n=1:1:N(i) %计算DFT频谱的实部和虚部
re=re+x(n)*cos(o*n);
im=im-x(n)*sin(o*n);
end
fou=sqrt(re.^2+im.^2);

%画由DFT得到的频谱及绘图设置
figure(2);
subplot(3,2,2*i);
plot(o,fou);
xlabel('\Omega/rad (数字角频率0-2*pi)');
ylabel('采样序列幅度谱|X(\Omega)|');
str=['1. ' num2str(f(i)) 'Hz 采样频谱(数字角频率)'];
title(str);
xlim([0,2*pi]);
end

%2
figure;%创建画布
O=0:0.01:2*pi; 
dtft=0;
re=0;
im=0;
O_32=0:pi/16:2*pi-pi/16;
re_32=0;
im_32=0;
O_16=0:pi/8:2*pi-pi/8;
re_16=0;
im_16=0;            %创建DTFT横坐标和采样频谱坐标及实部虚部数组
for n=0:1:26
    if n<=13        %n<14的情况
        re=re+(n+1)*cos(n*O);
        im=im-(n+1)*sin(n*O);       %计算DTFT
         re_32=re_32+(n+1)*cos(n*O_32);
        im_32=im_32-(n+1)*sin(n*O_32);      %对实部和虚部在DTFT谱上32点采样
         re_16=re_16+(n+1)*cos(n*O_16);
        im_16=im_16-(n+1)*sin(n*O_16);      %对实部和虚部在DTFT谱上16点采样
    end
    if n>13         %n>13的情况
        re=re+(27-n)*cos(n*O);
        im=im-(27-n)*sin(n*O);      %计算DTFT
         re_32=re_32+(27-n)*cos(n*O_32);
        im_32=im_32-(27-n)*sin(n*O_32);     %对实部和虚部在DTFT谱上32点采样
         re_16=re_16+(27-n)*cos(n*O_16);
        im_16=im_16-(27-n)*sin(n*O_16);         %对实部和虚部在DTFT谱上16点采样
    end
end
dtft=sqrt(re.^2+im.^2);
dft_32=sqrt(re_32.^2+im_32.^2);
dft_16=sqrt(re_16.^2+im_16.^2);     %计算DTFT幅度谱 32点采样频域幅度谱  16点采样频域幅度谱
subplot(3,2,1);         %画DTFT幅度谱
plot(O,dtft);
xlim([0,2*pi]);
xlabel('\Omega/rad  (数字角频率 0-2*pi)');
ylabel('|X(\Omega)|');
title('DTFT频谱');
subplot(3,2,3);     %画32点频域采样频谱
stem(O_32,dft_32);
xlim([0,2*pi]);
xlabel('k\Omega/rad (\Omega=2*pi/32)');
ylabel('|X32(k)|');
title('32点采样频谱');
subplot(3,2,5);         %画16点频域采样频谱
stem(O_16,dft_16);
title('16点采样频谱');
xlim([0,2*pi]);
xlabel('k\Omega/rad  (\Omega=2*pi/16)');
ylabel('|X16(k)|');

%画时域图
x=zeros(1,32);
n=1;
for i=0:1:26
    if i<=13
        x(n)=i+1;
    end
    if i>13
        x(n)=27-i;
    end
    n=n+1;
end
n=0:1:31;
subplot(3,2,2);         %画原信号
stem(n,x);
title('x(n)');
ylabel('x(n)');
xlabel('n');
xlim([0 31]);

x_32_re=zeros(1,32);            %计算由频域32点采样的道德IDFT x32(n)
x_32_im=zeros(1,32);
for n=1:1:32            
    for i=1:1:32        %计算IDFT
        x_32_re(n)= x_32_re(n)+re_32(i)*cos((i-1)*2*pi*(n-1)/32)/32-im_32(i)*sin((i-1)*2*pi*(n-1)/32)/32;
        x_32_im(n)= x_32_im(n)+im_32(i)*cos((i-1)*2*pi*(n-1)/32)/32+re_32(i)*sin((i-1)*2*pi*(n-1)/32)/32;
    end
end
subplot(3,2,4);
n=0:1:32-1;
stem(n,x_32_re);        %画32点IDFT 序列x32(n)
title('32点IDFT序列x32(n)');
ylabel('x32(n)');
xlabel('n');
xlim([0 31]);

x_16_re=zeros(1,16);
x_16_im=zeros(1,16);            %计算由频域16点采样的道德IDFT x16(n)
for n=1:1:16
    for i=1:1:16            %计算IDFT
        x_16_re(n)= x_16_re(n)+re_16(i)*cos((i-1)*2*pi*(n-1)/16)/16-im_16(i)*sin((i-1)*2*pi*(n-1)/16)/16;
        x_16_im(n)= x_16_im(n)+im_16(i)*cos((i-1)*2*pi*(n-1)/16)/16+re_16(i)*sin((i-1)*2*pi*(n-1)/16)/16;
    end
end
subplot(3,2,6);
n=0:1:16-1;
stem(n,x_16_re);            %画16点IDFT 序列x16(n)
title('16点IDFT序列x(n)');
ylabel('x16(n)');
xlabel('n');
xlim([0 15]);
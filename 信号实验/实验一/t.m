clear 
clc
%1
%整体绘制
x=0:0.001:8;
y=-square(x*pi/2); %创建矩形波序列和横坐标序列
figure;
subplot(2,2,1); %将画布四等分在指定位置绘图
plot(x ,y);
hold on;
fou1=-4/pi*sin(x*pi/2); %一次谐波
plot(x,fou1);
ylim([-1.5,1.5]);
xlim([0,8]); %绘图设置
title('1. 第一次谐波叠加');
legend('原信号','一次谐波叠加信号'); %设置标签
xlabel('t/s');
ylabel('f(t)');

%figure; %将画布四等分在指定位置绘图
subplot(2,2,2);
plot(x ,y)
fou2=0;
for i=1:2:5 %1 3 5次谐波叠加
fou2=fou2-4/pi/i*sin(i*x*pi/2);
end
hold on;
plot(x,fou2);
ylim([-1.5,1.5]); %绘图设置
xlim([0,8]); 
title('1. 第1,3,5次谐波叠加')
legend('原信号','第1,3,5次谐波叠加信号');
xlabel('t/s');
ylabel('f(t)');

%figure; %将画布四等分在指定位置绘图
subplot(2,2,3);
plot(x ,y)
fou3=0;
for i=1:2:49 %1-49次谐波叠加
fou3=fou3-4/pi/i*sin(i*x*pi/2);
end
hold on;
plot(x,fou3);
ylim([-1.5,1.5]); %绘图设置
xlim([0,8]);
title('1. 第1-49次谐波叠加')
legend('原信号','第1-49次谐波叠加信号');
xlabel('t/s');
ylabel('f(t)');

%figure; %将画布四等分在指定位置绘图
subplot(2,2,4);
plot(x ,y)
fou4=0;
for i=1:2:501 %1-501次谐波叠加
fou4=fou4-4/pi/i*sin(i*x*pi/2);
end
hold on;
plot(x,fou4);
ylim([-1.5,1.5]);
xlim([0,8]);
title('1. 第1-501次谐波叠加')
legend('原信号','第1-501次谐波叠加信号');
xlabel('t/s');
ylabel('f(t)');


%单独绘制
x=0:0.001:8;
y=-square(x*pi/2); %创建矩形波序列和横坐标序列
figure;
plot(x ,y);
hold on;
fou1=-4/pi*sin(x*pi/2); %一次谐波
plot(x,fou1);
ylim([-1.5,1.5]);
xlim([0,8]); %绘图设置
title('1. 第一次谐波叠加');
legend('原信号','一次谐波叠加信号'); %设置标签
xlabel('t/s');
ylabel('f(t)');

figure; 
plot(x ,y)
fou2=0;
for i=1:2:5 %1 3 5次谐波叠加
fou2=fou2-4/pi/i*sin(i*x*pi/2);
end
hold on;
plot(x,fou2);
ylim([-1.5,1.5]); %绘图设置
xlim([0,8]); 
title('1. 第1,3,5次谐波叠加')
legend('原信号','第1,3,5次谐波叠加信号');
xlabel('t/s');
ylabel('f(t)');

figure; 
plot(x ,y)
fou3=0;
for i=1:2:49 %1-49次谐波叠加
fou3=fou3-4/pi/i*sin(i*x*pi/2);
end
hold on;
plot(x,fou3);
ylim([-1.5,1.5]); %绘图设置
xlim([0,8]);
title('1. 第1-49次谐波叠加')
legend('原信号','第1-49次谐波叠加信号');
xlabel('t/s');
ylabel('f(t)');

figure; 
plot(x ,y)
fou4=0;
for i=1:2:501 %1-501次谐波叠加
fou4=fou4-4/pi/i*sin(i*x*pi/2);
end
hold on;
plot(x,fou4);
ylim([-1.5,1.5]);
xlim([0,8]);
title('1. 第1-501次谐波叠加')
legend('原信号','第1-501次谐波叠加信号');
xlabel('t/s');
ylabel('f(t)');


%2 第2种解法
T=4;  %
Ts=[0.01 0.02 0.05] %创建不同采样间隔数组
N=4./Ts;        %计算不同采样间隔一个周期采样数
c=60;
a1=zeros(1,c+1);
b1=zeros(1,c+1);
a2=zeros(1,c+1);
b2=zeros(1,c+1);
a3=zeros(1,c+1);
b3=zeros(1,c+1);  %创建采样序列频谱数组
%计算第一个采样间隔dfs
for k=0:c         %计算DFS
    K=k+1;
    for n=0:N(1)-1
        a1(K)=a1(K)-square(n*Ts(1)*pi/2)*cos(2*pi*k*n/N(1))/N(1);
        b1(K)=b1(K)+square(n*Ts(1)*pi/2)*sin(2*pi*k*n/N(1))/N(1);
    end
end
%计算第2个采样间隔dfs
for k=0:c         %计算DFS
    K=k+1;
    for n=0:N(2)-1
        a2(K)=a2(K)-square(n*Ts(2)*pi/2)*cos(2*pi*k*n/N(2))/N(2);
        b2(K)=b2(K)+square(n*Ts(2)*pi/2)*sin(2*pi*k*n/N(2))/N(2);
    end
end
%计算第3个采样间隔dfs
for k=0:c         %计算DFS
    K=k+1;
    for n=0:N(3)-1
        a3(K)=a3(K)-square(n*Ts(3)*pi/2)*cos(2*pi*k*n/N(3))/N(3);
        b3(K)=b3(K)+square(n*Ts(3)*pi/2)*sin(2*pi*k*n/N(3))/N(3);
    end
end

r1=zeros(1,2*N(1));
r2=zeros(1,2*N(2));
r3=zeros(1,2*N(3));    %创建由60次谐波复原的数组
%第一组采样间隔根据谐波复原
for n=0:2*N(1)-1         %计算由60次谐波复原的数组
    M=n+1;
    for k=0:c  %N-1
        K=k+1;
        r1(M)=r1(M)+a1(K)*cos(2*pi*k*n/N(1))-b1(K)*sin(2*pi*k*n/N(1));
    end
end
%第2组采样间隔根据谐波复原
for n=0:2*N(2)-1         %计算由60次谐波复原的数组
    M=n+1;
    for k=0:c  %N-1
        K=k+1;
        r2(M)=r2(M)+a2(K)*cos(2*pi*k*n/N(2))-b2(K)*sin(2*pi*k*n/N(2));
    end
end
%第3组采样间隔根据谐波复原
for n=0:2*N(3)-1         %计算由60次谐波复原的数组
    M=n+1;
    for k=0:c  %N-1
        K=k+1;
        r3(M)=r3(M)+a3(K)*cos(2*pi*k*n/N(3))-b3(K)*sin(2*pi*k*n/N(3));
    end
end

xx1=0:Ts(1):8-Ts(1);
xx2=0:Ts(2):8-Ts(2);
xx3=0:Ts(3):8-Ts(3);  %不同包络线时间尺度
sam1=0:1:2*N(1)-1;
sam2=0:1:2*N(2)-1;
sam3=0:1:2*N(3)-1;

%整体效果预览
figure;  %创建画布
%第一个采样间隔在画布第一行
subplot(3,3,1);  %在画布第一列绘制由plot(.)的散点图
plot(sam1,r1,'.');
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(1)-1)]);
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

subplot(3,3,2);
stem(sam1,r1);        %在画布第二列绘制由stem()的散点图
ylim([-1.5,1.5]);       %绘图设置
xlim([0,(2*N(1)-1)]);
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

subplot(3,3,3);     %在画布第三列绘制包络线
plot(xx1,r1);
ylim([-1.5,1.5]);   %绘图设置
xlim([0,8]);
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');

%第2个采样间隔在画布第2行
subplot(3,3,4);     %在画布第一列绘制由plot(.)的散点图
plot(sam2,r2,'.');
ylim([-1.5,1.5]);       %绘图设置
xlim([0,(2*N(2)-1)]);
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

subplot(3,3,5);     %在画布第二列绘制由stem()的散点图
stem(sam2,r2);     %绘图设置
ylim([-1.5,1.5]);
xlim([0,(2*N(2)-1)]);
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

subplot(3,3,6);      %在画布第三列绘制包络线
plot(xx2,r2);
ylim([-1.5,1.5]);       %绘图设置
xlim([0,8]);
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');

%第3个采样间隔在画布第3行
subplot(3,3,7);         %在画布第1列绘制由plot(.)的散点图
plot(sam3,r3,'.');     %绘图设置
ylim([-1.5,1.5]);
xlim([0,(2*N(3)-1)]);
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

subplot(3,3,8);     %在画布第2列绘制由stem()的散点图
stem(sam3,r3);
ylim([-1.5,1.5]);            %绘图设置
xlim([0,(2*N(3)-1)]);
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

subplot(3,3,9);     %在画布第3列绘制包络线
plot(xx3,r3);
ylim([-1.5,1.5]);       %绘图设置
xlim([0,8]);
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');


%第一问分别绘制
%用plot('.')绘制
figure;  %创建画布
%第一个采样间隔
plot(sam1,r1,'.');
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(1)-1)]);
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

figure;  %创建画布
%第2个采样间隔
plot(sam2,r2,'.');
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(2)-1)]);
 %
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

figure;  %创建画布
%第3个采样间隔
plot(sam3,r3,'.');
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(3)-1)]);
 %
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

%第一问用plot(.)绘制整体效果
figure;  %创建画布
%第1个采样间隔在画布第一列
subplot(1,3,1);
plot(sam1,r1,'.');
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(1)-1)]);
 %
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

%第2个采样间隔在画布第2列
subplot(1,3,2);
plot(sam2,r2,'.');
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(2)-1)]);
 %
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

%第3个采样间隔在画布第一列
subplot(1,3,3);
plot(sam3,r3,'.');
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(3)-1)]);
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波合成图像 用plot(.)绘制'];
title(str);
xlabel('n');
ylabel('x(n)');





%第一问分别绘制
%用stem()绘制
figure;  %创建画布
%第一个采样间隔
stem(sam1,r1);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(1)-1)]);
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

figure;  %创建画布
%第2个采样间隔
stem(sam2,r2);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(2)-1)]);
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

figure;  %创建画布
%第3个采样间隔
stem(sam3,r3);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(3)-1)]);
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

%第一问用stem()绘制整体效果
figure;  %创建画布
%第1个采样间隔在画布第一列
subplot(1,3,1);
stem(sam1,r1);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(1)-1)]);
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

%第2个采样间隔在画布第2列
subplot(1,3,2);
stem(sam2,r2);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(2)-1)]);
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');

%第3个采样间隔在画布第一列
subplot(1,3,3);
stem(sam3,r3);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,(2*N(3)-1)]);
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波合成图像 用stem()绘制'];
title(str);
xlabel('n');
ylabel('x(n)');


%第二问
figure;  %创建画布
%第一个采样间隔
plot(xx1,r1);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,8]);
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');

figure;  %创建画布
%第2个采样间隔
plot(xx2,r2);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,8]);
 
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');

figure;  %创建画布
%第3个采样间隔
plot(xx3,r3);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,8]);
 
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');

%第二问整体效果
figure;  %创建画布
%第1个采样间隔在画布第一列
subplot(1,3,1);
plot(xx1,r1);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,8]);
 
str=['2. 取样间隔',num2str(Ts(1)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');

%第2个采样间隔在画布第2列
subplot(1,3,2);
plot(xx2,r2);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,8]);
 
str=['2. 取样间隔',num2str(Ts(2)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');

%第3个采样间隔在画布第一列
subplot(1,3,3);
plot(xx3,r3);
ylim([-1.5,1.5]);  %绘图设置
xlim([0,8]);
 
str=['2. 取样间隔',num2str(Ts(3)),' 60次谐波包络线'];
title(str);
xlabel('t/s');
ylabel('x(n)');

%3
figure;     %创建图层
x=0:0.001:8;
f=sawtooth(pi*x/2);  %锯齿波信号
plot(x,f);
ylim([-1.5,1.5]);
xlim([0,8]);  %画出原锯齿波信号
hn=7;
N=8000;
Ts=4/(N-1);
b=zeros(1,N);        %锯齿波奇函数
for k=1:hn          %计算七次谐波的bn
    for n=0:N-1
        b(k)=b(k)+2*sawtooth(pi*n*Ts/2)*sin(pi/2*k*n*Ts)/N;
    end
end
fou=0;
for i=1:hn          %计算七次谐波叠加信号
    fou=fou+b(i)*sin(pi/2*i*x);
end
hold on;
plot(x,fou);        %绘制七次谐波叠加信号
legend('原信号','合成信号');
str=['3. ',num2str(hn),'次谐波合成信号与原信号对比'];
title(str);


 



clear 
clc

num=[0 0 1];
den=[6 12 3 1];
T=200;
TS=0.00001;
sys=tf(num,den);
 
step(sys,[0:0.001:50]);

hold;
a=normrnd(0,0.01,1,T/TS+1);


t=0:TS:T;
u=ones(1,T/TS+1);

y1=lsim(sys,u,t);
c1=0;

for i=0:1:T/TS
     
   c1=c1+TS*(1-y1(i+1)-a(i+1));
end

den1=[c1 1];
y2=lsim(tf(num,den1),u,t);


c2=0;
z=zeros(1,T/TS+1);

for j=1:1:T/TS
        z(j+1)=z(j)+(y2(j)-y1(j))*TS;
end

for j=0:1:T/TS
    c2=c2+z(j+1)*TS;
end


   
den2=[c2 c1 1];
y3=lsim(tf(num,den2),u,t);


c3=0;
y4=zeros(1,T/TS+1);
y5=zeros(1,T/TS+1);
for k=1:1:T/TS
    y4(k+1)=y4(k)+(y3(k)-y1(k))*TS;
end
for j=1:1:T/TS
    y5(j+1)=y5(j)+y4(j)*TS;
end
for i=0:1:T/TS
    c3=c3+y5(i+1)*TS;
end      

den3=[c3 c2 c1 1]
sys1= tf(num , den3);
step(sys1,[0:0.001:50]);



str=['辨识曲线，噪声方差为' num2str(0.01)];

legend('原始响应曲线',str);


l=1;
d=2;
n=100000;
count=0;
for i=1:n
    x=d/2*rand();
    ang=pi/2*rand();
    if x<=l/2*sin(ang)
        count=count+1;
    end
end
p=count/n;
disp('模拟10000次')
fprintf('投针求得的pi为:%f\n',1/p)

count=0;
for i=1:n
    x=rand();
    y=rand();
    dis=sqrt(x*x+y*y);
    if dis<1
        count=count+1;
    end
end
p=count/n;
fprintf('蒙特卡洛求得的pi为:%f\n',4*p)
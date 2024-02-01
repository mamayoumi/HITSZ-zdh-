N=100000;
M=10000;
sum=zeros(1,M);
v=zeros(1,M);
for j=1:M
for i=1:N
    x=rand();
    sum(j)=sum(j)+x;
end
v(j)=(sum(j)-N/2)/sqrt(N/12);
end

histogram(v,20,'Normalization','pdf');
title('直方图概率分布');
figure;
k=1:M;

%length(k)
%length(v)
plot(k,v,'r');

k=k*20/N;
xlabel('k');
ylabel('v');
title('N(0,1)正态分布白噪声');


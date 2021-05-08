clc
close all
clear all
disp('���˻��Ż�ģ��')
%����׼��
%�����û�λ��
d1=100; %����A����
d2=80; %����A����
N=30;%�û�����
C=6;%���߸���
bc=1;%������/ÿ������---����������� ���������ͬ
Bm=[10 10 10];%���˻�����
%����λ��tuple={content k,UAV m��location q}
height=10;
h_max=20;
delta_d=0.5;%location interval
n1=3;%���˻�����
%discrete the location
%�û�λ��
%discrete the location
d1a=0:delta_d:d1;
d2a=0:delta_d:d2;
%��ɢλ��
index=[];
for i=1:1:length(d1a)
    for j=1:1:length(d2a)
        index=[index;[d1a(i),d2a(j)]];
    end
end
num_index=size(index,1);%number of location
%������Ⱥλ�ü���
%������Ⱥ�����ܳ���λ��
d1a1=[5,d1-5];
d2a1=[5,d2-5];
[len,~]=find((index(:,1)>=d1a1(1) & index(:,1)<=d1a1(2)) & (index(:,2)>=d2a1(1) & index(:,2)<=d2a1(2)));
id_new=setdiff(1:1:size(index,1),len');  %�����û����
new_candi=index(id_new',:);%������Ⱥλ��
%��������û�
user_index=randperm(num_index,N);
user_location=[];
for i=1:1:length(user_index)
    user_location=[user_location;index(user_index(i),:)];%%N*2
end
%����ʱ��T
T=5;%����ʱ��T
v=1;%�û������ٶ�
delta_p=2;%��λʱ��仯����
theta_max=2*pi;%�û�������߷���
best_uav=cell(1,T);
best_task=zeros(1,T);
[x_ind,task]=one_step(user_location,height,h_max,C,N,n1,bc,Bm);  %������
best_uav{1,1}=x_ind;
best_task(1,1)=task;
old_ind=x_ind;%���þ͵����˻�λ��
for t=2:1:T
    %�����û�
    new_user_location=[];
    %����ԭʼ�û�����
    for i=1:1:size(user_location,1) 
      user_location(i,1)=user_location(i,1)+v*cos(random('unif',0,theta_max));
      user_location(i,2)=user_location(i,2)+v*sin(random('unif',0,theta_max));
      %�߽���£���������߽� ����Ϊ�û��뿪
      if (user_location(i,1)>d1) | (user_location(i,1)<0) | (user_location(i,2)<0)|(user_location(i,2)>d2)
          continue;%�û��뿪����
      else
          new_user_location=[new_user_location;user_location(i,:)]; %�û�û�뿪����
      end
    end
    %��ǰʱ�������ͻ�
    num_new=random('poiss',delta_p);
    if num_new==0
        new_user_location=new_user_location;
    else
        new_id=randperm(length(id_new),num_new);
        new_coming=new_candi(new_id',:);
        new_user_location=[new_user_location; new_coming];
    end
    %�û���������
    N=size(new_user_location,1);
    %���㵱ǰ���˻�λ��
    [x_ind,task]=one_step(new_user_location,height,h_max,C,N,n1,bc,Bm);  %������
    %���˻�λ�÷���--���ķ����������˻��������˻�����λ��
    x_ind=allot_uav(x_ind,old_ind);
    best_uav{1,t}=x_ind;
    best_task(1,t)=task;
    %����һʱ���û�λ�ø���
    user_location=new_user_location;%��һʱ���û�λ��
    old_ind=x_ind;
    if mod(t,20)==0
    figure,
    plot(user_location(:,1),user_location(:,2),'r*')
    xlabel('x')
    ylabel('y')
    title(num2str(t))
    end
end
figure,plot(1:1:T,best_task,'->')
xlabel('T/(min)')
ylabel('Task/(n)')
grid on
ylim([0 40])
Q1=[];Q2=[];Q3=[];
for i=1:1:T
Q1=[Q1,(best_uav{1,i}(1,:))'];
Q2=[Q2,(best_uav{1,i}(2,:))'];
Q3=[Q3,(best_uav{1,i}(3,:))'];
end
figure,
plot3(Q1(1,:),Q1(2,:),Q1(3,:),'r->')
hold on
plot3(Q2(1,:),Q2(2,:),Q2(3,:),'b->')
hold on
plot3(Q3(1,:),Q3(2,:),Q3(3,:),'c->')
xlabel('x')
ylabel('y')
zlabel('height')
grid on
legend('UAV-1','UAV-2','UAV-3')
title('the location of UAV')


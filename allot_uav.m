function x_in=allot_uav(x,y)
%x��ʾ�µ�UAVλ��(x1,y1,z1)
%y��ʾ�ɵ�UAVλ��(x2,y2,z2)
member=[];
for i=1:1:size(x,1)
    new=x(i,:);
    for j=1:1:size(y,1)
        if ~any(j==member)
           dist(j)=sqrt(sum((new-y(j,:)).^2));
        else
           dist(j)=0;
        end
    end
    [~,len]=find(0~=dist); %�жϲ�Ϊ��
    dist1=dist(len);%�Ѳ�Ϊ��ĸ���
    [~,len1]=find(min(dist1)==dist1); %�жϲ�Ϊ��
    %�ҳ���̾���
    x_in(len(len1),:)=new;
    member=[member,len(len1)];
end
end
h_profiles=zeros(1,size(R,1)+size(R,2));

for i = 1:size(R,1)
     h_profiles(i)=std(R(i,:));
     h_profiles(size(R,1)+i)=std(R(:,i));
end;

        
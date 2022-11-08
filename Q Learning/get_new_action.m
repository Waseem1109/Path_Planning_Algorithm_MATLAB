function A_I=get_new_action(Q,row,column,epsilon)
if(rand(1,1)<epsilon)
    d=Q(row,column,:);
    A_I=find(d==max(d));
    A_I=A_I(1);
else
    A_I=randi(4,1);
end  
end
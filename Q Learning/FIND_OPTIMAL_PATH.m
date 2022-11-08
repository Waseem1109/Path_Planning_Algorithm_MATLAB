function path=FIND_OPTIMAL_PATH(StartPoint,epsilon,Q,actions,Rewards)
S=size(Rewards);
S_R_I=S(1)-StartPoint(2);S_C_I=StartPoint(1);
if(isOccupied(S_R_I,S_C_I,Rewards))
    path=[];
else
    C_R_I=S_R_I;C_C_I=S_C_I;
    clear path;
    path(1,:)=[C_C_I S(1)-C_R_I];
    k=1;
    while(isOccupied(C_R_I,C_C_I,Rewards)==0)
        k=k+1;
        A_I=get_new_action(Q,C_R_I,C_C_I,epsilon);
        O_R_I=C_R_I;O_C_I=C_C_I;
        [C_R_I,C_C_I]=goto_location(O_R_I,O_C_I,actions,A_I);
        path(k,:)=[C_C_I S(1)-C_R_I];
    end
end
end
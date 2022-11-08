function [C_R_I,C_C_I]=NonOccupied_random_location(Rewards)
C_R_I=randi([1 14],1);C_C_I=randi([1 20],1);
while(isOccupied(C_R_I,C_C_I,Rewards))
    C_R_I=randi(14,1);C_C_I=randi(20,1);
end
end
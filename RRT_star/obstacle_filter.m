function [distance,ob3]=obstacle_filter(distance,theta_C,thetaUltra,x_C,y_C,Obstacles,dthresh,obstacle_thickness)
p=2.5;
r=15;
x00 = x_C + (distance + p) * cosd(thetaUltra + theta_C - 90) + r * cosd(theta_C);
y00 = y_C + (distance + p) * sind(thetaUltra + theta_C - 90) + r * sind(theta_C);
LL=obstacle_thickness*2;
ob3=[x00-obstacle_thickness y00-obstacle_thickness LL LL];
dh=size(Obstacles);
for o=1:1:dh(1,1)
    ob=[Obstacles(o,1)+obstacle_thickness Obstacles(o,2)+obstacle_thickness];
    Exb=sqrt((ob(1)-x00)^2+((ob(2)-y00)^2));
    if(Exb<dthresh)
        distance = 200;
        ob3=[];
    end
end
end
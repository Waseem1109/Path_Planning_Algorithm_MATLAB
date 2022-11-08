function PLOTBOT(x,y,r,theta,p,thetaUltra)
plot(x,y,'ro','LineWidth',2)
x_1=x+r*cosd(theta);y_1=y+r*sind(theta);
plot([x,x_1],[y,y_1],'r-','LineWidth',2);
plot([x-13*cosd(theta)-9*sind(theta),x+20*cosd(theta)-9*sind(theta)],[y+9*cosd(theta)-13*sind(theta),y+9*cosd(theta)+20*sind(theta)],'g-')
plot([x-13*cosd(theta)+9*sind(theta),x+20*cosd(theta)+9*sind(theta)],[y-9*cosd(theta)-13*sind(theta),y-9*cosd(theta)+20*sind(theta)],'g-')
plot([x-9*sind(theta+180)-13*cosd(theta),x+9*sind(theta+180)-13*cosd(theta)],[y-9*cosd(theta)-13*sind(theta),y+9*cosd(theta)-13*sind(theta)],'g-')
plot([x-9*sind(theta+180)+20*cosd(theta),x+9*sind(theta+180)+20*cosd(theta)],[y-9*cosd(theta)+20*sind(theta),y+9*cosd(theta)+20*sind(theta)],'g-')
plot([x_1,x_1+p*cosd(thetaUltra+theta-90)],[y_1,y_1+p*sind(thetaUltra+theta-90)],'b-','LineWidth',2)
end
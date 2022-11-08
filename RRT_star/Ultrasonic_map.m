function map = Ultrasonic_map(map,distance,x_C,y_C,theta_C,p,r,thetaUltra,threshhold)
if (distance < threshhold && distance>5)
    x1 = x_C + (distance + p) * cosd(thetaUltra + theta_C - 90) + r * cosd(theta_C);
    y1 = y_C + (distance + p) * sind(thetaUltra + theta_C - 90) + r * sind(theta_C);
    x2=x1-(1);x3=x1+(1);
    y2=y1-(1);y3=y1+(1);
    for bx=x2:1:x3
        for by=y2:1:y3
            setOccupancy(map, [bx by], ones(1, 1));
        end
    end
end
end
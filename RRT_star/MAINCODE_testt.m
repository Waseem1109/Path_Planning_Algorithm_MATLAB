clc
clear
close all
tic
%% Build Bridge b/w Arduino and MATLAB serial com
s = serialport("COM3", 115200, "Timeout", 20);
x_size = 200;
y_size = 200;
map = binaryOccupancyMap(x_size, y_size, 1);
%% obtsacle finding parameters
obstacle_thickness = 10;
prevtime = 0;
dthresh = 2;
threshhold = 70;
thickness = 10;
thetaUltra = 90;

%% Robot Parameters
p = 2.5;
r = 15;
%% Indicator and Scure message variable
N = 0;
M = 1234;
LP = 0;
RP = 0;
i=0;
%% Initial and Final Point decalaration
x_C = 25; y_C = 0; theta_C = pi / 2;
x_N=input("Enter Final X point (cm) = ");
y_N=input("Enter Final Y point (cm) = ");
theta_N=input("Enter Final orientation (radian) = ");
Obstacles = [];
CurrentPoint = [x_C y_C theta_C];
GoalPoint = [x_N y_N theta_N];
%% Sending and Recieving parameters and SendRecieve service for servo rotates plus mapping
data = zeros(1, 11);
data(11) = 1;
point = [0 0];

while (data(11) ~= 0)
    flush(s);
    write(s, [point theta_N M RP LP], 'single');
    data = read(s, 13, 'single');
    disp(data);
    
    if (data(8) == 1234)
        x_C = data(1);
        y_C = data(2);
        theta_C = data(3) * 57.29;
        distance = data(7); thetaUltra = data(10);
        [distance,ob3] = obstacle_filter(distance, theta_C, thetaUltra, x_C, y_C, Obstacles, dthresh,obstacle_thickness);
        [map,Obstacles] = Ultrasonic_map1(map,distance,threshhold,Obstacles,ob3,obstacle_thickness);
    else
        data(11) = 1;
    end
    if ((toc - prevtime) > 1)
        show(map)
        hold on
        PLOTBOT(x_C, y_C, r, theta_C, p, thetaUltra);
        prevtime = toc;
    end
    
end



%% planning paramerters
obstacle_thickness =7.5;
stepsize = 10;
max_iteration = 300;
Connecting_radius = 20;
%% Path Plan
path = PLANNER_RRT(x_size, y_size, stepsize, max_iteration, Obstacles, CurrentPoint, GoalPoint, Connecting_radius);
%% PLOT
show(map)
hold on
plot(path(:, 1), path(:, 2), 'Color', 'm', 'LineWidth', 2)
PLOTBOT(x_C, y_C, r, theta_C, p, thetaUltra);
%% start
PATH = path;
Turns = size(PATH);
ErrorX = 100;
ddthresh=20;dd1thresh=7;dd2thresh=7;
v(1, 1) = 0; % actual path taken
EL_thresh = 4;
ES_thresh = 4;

while (N < Turns(1, 1))
    N = N + 1;
    
    if (N == Turns(1, 1))
        EL_thresh = 2;
        ES_thresh = 2;
    end
    
    flush(s);
    point(1, :) = PATH(N, :);
    write(s, [point theta_N M RP LP], 'single');
    data = read(s, 13, 'single');
    disp(data);
    
    if (data(8) == 1234)
        i = i + 1;
        x_C = data(1);
        v(i, 1) = x_C;
        y_C = data(2);
        v(i, 2) = y_C;
        theta_C = data(3) * 57.296;
        distance = data(7); thetaUltra = data(10);
        [distance,ob3] = obstacle_filter(distance, theta_C, thetaUltra, x_C, y_C, Obstacles, dthresh,obstacle_thickness);
        
        if (data(12) == 1)
            RP = 0;
        end
        
        ErrorX = sqrt((x_C - point(1, 1))^2 + (y_C - point(1, 2))^2);
        [map,Obstacles] = Ultrasonic_map1(map,distance,threshhold,Obstacles,ob3,obstacle_thickness);
    end
    
    while ((ErrorX > EL_thresh))
        
        while ((ErrorX > ES_thresh))
            flush(s);
            write(s, [point theta_N M RP LP], 'single');
            data = read(s, 13, 'single');
            disp(data);
            
            if (data(8) == 1234)
                i = i + 1;
                x_C = data(1);
                v(i, 1) = x_C;
                y_C = data(2);
                v(i, 2) = y_C;
                theta_C = data(3) * 57.296;
                distance = data(7); thetaUltra = data(10);
                [distance,ob3] = obstacle_filter(distance, theta_C, thetaUltra, x_C, y_C, Obstacles, dthresh,obstacle_thickness);
                
                if (data(12) == 1)
                    RP = 0;
                end
                
                ErrorX = sqrt((x_C - point(1, 1))^2 + (y_C - point(1, 2))^2);
                [map,Obstacles] = Ultrasonic_map1(map,distance,threshhold,Obstacles,ob3,obstacle_thickness);
                ErrP=sqrt((x_C-x_N)^2+(y_C-y_N)^2);
                if(ErrP<10)
                    ddthresh=5;
                end
                if (distance < ddthresh && distance > 5 && data(11) == 0 && data(12) == 0 && RP == 0)
                    show(map)
                    PLOTBOT(x_C, y_C, r, theta_C, p, thetaUltra);
                    plot(path(:, 1), path(:, 2), 'Color', 'm', 'LineWidth', 2)                 
                    RP = 1;
                    CurrentPoint = [x_C y_C theta_C];
                    clear pthObj solnInfo
                    path = PLANNER_RRT(x_size, y_size, stepsize, max_iteration, Obstacles, CurrentPoint, GoalPoint, Connecting_radius);
                    PATH = path;
                    Turns = size(PATH);
                    N = 1;
                    point(1, :) = PATH(N, :);
                end
                
            end
            
            if ((toc - prevtime) > 2)
                show(map)
                PLOTBOT(x_C, y_C, r, theta_C, p, thetaUltra);
                plot(path(:, 1), path(:, 2), 'Color', 'm', 'LineWidth', 2)
                prevtime = toc;
            end
            
        end
        
    end
    
end

LastAngle=theta_N*57.296;
Err_theta=abs(LastAngle-theta_C);
LP=1;
for j=1:1:3
    Err_theta=100;
    while (Err_theta>(0.07*57.296))
        flush(s);
        point(1, :) = PATH(Turns(1,1), :);
        write(s, [point theta_N M RP LP], 'single');
        data = read(s, 13, 'single');
        disp(data);
        if(data(8)==1234)
            i=i+1;
            x_C = data(1);
            v(i,1)=x_C;
            y_C = data(2);
            v(i,2)=y_C;
            theta_C = data(3) * 57.296;
            distance=data(7);thetaUltra=data(10);
            [distance,ob3] = obstacle_filter(distance, theta_C, thetaUltra, x_C, y_C, Obstacles, dthresh,obstacle_thickness);
            ErrorX = sqrt((x_C - point(1,1))^2+(y_C - point(1,2))^2);
            Err_theta=abs(LastAngle-theta_C);
            [map,Obstacles] = Ultrasonic_map1(map,distance,threshhold,Obstacles,ob3,obstacle_thickness);
        end
        while ((Err_theta > (0.07*57.296)))
            flush(s);
            write(s, [point theta_N M RP LP], 'single');
            data = read(s, 13, 'single');
            disp(data);
            if(data(8)==1234)
                i=i+1;
                x_C = data(1);
                v(i,1)=x_C;
                y_C = data(2);
                v(i,2)=y_C;
                theta_C = data(3) * 57.29;
                Err_theta=abs(LastAngle-theta_C);
                distance=data(7);thetaUltra=data(10);
                [distance,ob3] = obstacle_filter(distance, theta_C, thetaUltra, x_C, y_C, Obstacles, dthresh,obstacle_thickness);
                ErrorX = sqrt((x_C - point(1,1))^2+(y_C - point(1,2))^2);
                [map,Obstacles] = Ultrasonic_map1(map,distance,threshhold,Obstacles,ob3,obstacle_thickness);
            end
            if ((toc - prevtime) > 5)
                show(map)
                PLOTBOT(x_C, y_C, r, theta_C, p, thetaUltra);
                plot(path(:, 1), path(:, 2), 'Color', 'm', 'LineWidth', 2)
                prevtime = toc;
            end
        end
    end
end
hold off
show(map)
hold on
PLOTBOT(x_C, y_C, r, theta_C, p, thetaUltra);
plot(path(:, 1), path(:, 2), '-g');
plot(v(:, 1), v(:, 2), '.b')

delete(s);

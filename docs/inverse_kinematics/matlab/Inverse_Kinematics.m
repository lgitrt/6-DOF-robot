%Inverse Kinematics Calculation for Luca Robotic arm
%Author: Luca Obwegs
%Verison: 0.0
clc; clear all;
close all;

%syms phi1 phi2 phi3 phi4 phi5 phi6 %rotation angles
%syms d a c f h i %translations
%d = 154; a = 27.36; c = 177.18; f = 3.47; h = 140.98; i = 10;


%Calculate the tranformation from TCP to world matrix using denavit hartenberg:
%T_bruch = getTz(phi1, d)*getTx(pi/2, -a)*getTz(phi2, 0)*getTx(0,c)...
%    *getTz((pi/2-phi3),0)*getTx(-pi/2,f)*getTz(phi4,h)*getTx(pi/2,0)...
%    *getTz(phi5,0)*getTx(-pi/2,0)*getTz((pi/2+phi6), i)*getTx(90,0);

%T_gk = vpa(T_bruch, digits);

%to calculate the inverse kinematics of a 6dof robotic arm, we need to
%divide our robot into 2 parts, Part1: joint 1 2 3 (responsible for positioning)
%Part2: joint 4 5 6 (responsible for rotation)

%Inverse kinematics of Part1 Joint 1 2 3 to get T03
%T03 = vpa(simplify(getTz(phi1, d)*getTx(pi/2, -a)*getTz(phi2, 0)*getTx(0,c)...
%    *getTz((pi/2-phi3),0)*getTx(-pi/2,f)*getTz(0,h)), 4)

%T03 = [[-sin(phi2 - phi3)*cos(phi1), -sin(phi1), -cos(phi2 - phi3)*cos(phi1), -cos(phi1)*(a - c*cos(phi2) + h*cos(phi2 - phi3) + f*sin(phi2 - phi3))]
%       [-sin(phi2 - phi3)*sin(phi1),  cos(phi1), -cos(phi2 - phi3)*sin(phi1), -sin(phi1)*(a - c*cos(phi2) + h*cos(phi2 - phi3) + f*sin(phi2 - phi3))]
%       [           cos(phi2 - phi3),          0,           -sin(phi2 - phi3),              d + c*sin(phi2) + f*cos(phi2 - phi3) - h*sin(phi2 - phi3)]
%       [                          0,          0,                           0,                                                                      1]];
yt = -200; zt = 200;
x0 = 0; y0 = 0; z0 = 0;
p3 = [yt;zt]';
p0 = [y0;z0]';
a=27.36; d=154; c = 177.18; f = 3.47; h = 140.98;

m = sqrt(h^2+f^2);
phi3 = pi - acos((c^2 - (a + yt)^2 + m^2 - (d - zt)^2)/(2*c*m));
phi2 = -atan((sin(phi3)*m)/(c + cos(phi3)*m)) - atan((d - zt)/(a + yt));

if(phi2<0 && yt<10)
    phi2 = phi2+pi;
end

%vectors
v02 = [-a; d];
v23 = [c*cos(phi2); c*sin(phi2)];
v34 = [m*cos(phi3+phi2); m*sin(phi3+phi2)];
v = v02+v23+v34;

%plot
fh = figure(); hold on; axis equal;
plot([0 v02(1)],[0 v02(2)],'LineWidth',5, 'Color', 'black');
xc = [v02(1) v02(1)+v23(1)];
yc = [v02(2) v02(2)+v23(2)];
xd = [v02(1)+v23(1) v02(1)+v23(1)+v34(1)];
yd = [v02(2)+v23(2) v02(2)+v23(2)+v34(2)];
x0 = [0 yt];
y0 = [0 zt];
B = plot(x0,y0,'LineWidth',5, 'Color', 'yellow');
C = plot(xc,yc,'LineWidth',5, 'Color', 'green');
D = plot(xd,yd,'LineWidth',5, 'Color', 'blue');
set(B, 'XDataSource', 'x0', 'YDataSource', 'y0')
set(C, 'XDataSource', 'xc', 'YDataSource', 'yc')
set(D, 'XDataSource', 'xd', 'YDataSource', 'yd')
refreshdata(fh, 'caller');


for i=1:1:400
    yt = yt+1;
    zt = zt+0.1;
    x0 = [0 yt];
    y0 = [0 zt];
    phi3 = pi - acos((c^2 - (a + yt)^2 + m^2 - (d - zt)^2)/(2*c*m));
    phi2 = -atan((sin(phi3)*m)/(c + cos(phi3)*m)) - atan((d - zt)/(a + yt));
    
    if(phi2<0 && yt<10)
        phi2 = phi2+pi;
    end
    
    %vectors
    v23 = [c*cos(phi2); c*sin(phi2)];
    v34 = [m*cos(phi3+phi2); m*sin(phi3+phi2)];

    xc = [v02(1) v02(1)+v23(1)];
    yc = [v02(2) v02(2)+v23(2)];
    xd = [v02(1)+v23(1) v02(1)+v23(1)+v34(1)];
    yd = [v02(2)+v23(2) v02(2)+v23(2)+v34(2)];
    
    %plot
    refreshdata(fh, 'caller');
    pause(0.001)
end

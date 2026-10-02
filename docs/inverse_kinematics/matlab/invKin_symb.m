%Symbolic derivation of the joint 2/3 inverse kinematics solution
%6-DOF Robotic Arm - Author: Luca Obwegs
clc; clear all;
syms d a c f h i %translations
syms phi1 phi2 phi3 %angles
syms yt zt xt %coordinates

%m
m = sqrt(h^2+f^2);
g = sqrt((a+yt)^2+(zt-d)^2);

%phi3
phi3 = simplify(acos((g^2-c^2-m^2)/(2*c*m)))

v = atan((zt-d)/(a+yt));
del = atan((m*sin(phi3))/(c+m*cos(phi3)));

%phi2
phi2 = simplify(v-del)


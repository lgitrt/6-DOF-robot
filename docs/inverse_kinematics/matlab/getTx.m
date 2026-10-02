function [T] = getTx(phix,xt)
%GETTRANSFORMATIONMATRIX Return transformation matrix, screw in x direction
%6-DOF Robotic Arm - Author: Luca Obwegs
    T = [1 0 0 xt;
         0 cos(phix) -sin(phix) 0;
         0 sin(phix) cos(phix) 0;
         0 0 0 1];
end


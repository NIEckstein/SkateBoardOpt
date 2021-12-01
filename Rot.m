function [outputArg1] = Rot(type,ang)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if type == 1
outputArg1 = [1 0 0;0 cos(ang) -sin(ang);0 sin(ang) cos(ang)];
elseif type == 2
     outputArg1 = [cos(ang) 0 sin(ang);0 1 0;-sin(ang) 0 cos(ang)]; 
    elseif type == 3
            outputArg1 = [cos(ang) -sin(ang) 0;sin(ang) cos(ang) 0;0 0 1];
        end
    end

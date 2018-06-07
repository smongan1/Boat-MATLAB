function [ angle ] =arctan( x,y )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
if (x==0&&sign(y)==1)
    angle=pi/2;

elseif (x==0)
    angle=3*pi/2;

else
    if (sign(x)==sign(y))
        if(sign(y)==-1)
            angle=atan(y/x)+pi;
           
        else
            angle=atan(y/x);

        end
    else
        if(sign(x)==-1)
            angle=atan(y/x)+pi;

        else
            angle=2*pi+atan(y/x);

        end
    end
end

end


function [ OutputPoints ] = PlaneMesh( Arg )
%PLANEMESH Create a planar grid of points centered at (0,0).
%
% Input : Arg arrow with at position
% 1) 0.5*width of the plane
% 2) 0.5*length of the plane
% 3) point density in x
% 4) point density in y
%
% Output :
%   TrueDataPoints : the true set of points from the generated shape.
%   
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 24 mai 2011

%Set the defaults if necessary
if size(Arg,2)<4
    if size(Arg,2)<3
        if size(Arg,2)<2
            if size(Arg,2)<1
                Arg(1) = 10;
            end
            Arg(2) = 10;
        end
        Arg(3) = 1; 
    end
    Arg(4) = 1;
end

plsizex=floor(2*Arg(1)/Arg(3))+1;
plsizey=floor(2*Arg(2)/Arg(4))+1;
OutputPoints = zeros(3, plsizex*plsizey);
nbp = 0;
for x=-Arg(1):Arg(3):Arg(1)
    for y=-Arg(2):Arg(4):Arg(2)
        nbp = nbp+1;
        OutputPoints(:,nbp)=[x,y,0]';
    end
end

end


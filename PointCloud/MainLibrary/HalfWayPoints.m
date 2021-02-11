function OutputPoints = HalfWayPoints( InputPoints )
% HALFWAYPOINTS Loop on all the possible pairs in the input points and compute the half way point. 
% Returns then the original list imcremented with the generated points.
%
% Input : 
%   InputPoints : n 3D points in the form of a (3,n) matrix
%
% Output :
%   OutputPoints : simulated data points with errors
%
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.1
%STATUS  : OK

s = size( InputPoints(1,:),2 );
OutputPoints = [ InputPoints, zeros(3,factorial(s-1)) ];

nb = 0;
for ip=1:s
    jp = ip+1;
    while( jp <= s )
        nb = nb+1;  
        OutputPoints(:,s+nb) = (InputPoints(:,ip) + InputPoints(:,jp))./2;
       jp = jp+1; 
    end
end

end

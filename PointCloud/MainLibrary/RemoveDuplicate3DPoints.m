function [ Points ] = RemoveDuplicate3DPoints( Points )
%REMOVEDUPLICATE3DPOINTS Remove the duplicates in a set of 3D points.
%
% Input : 
%   Points : 3xn or nx3 matrix of n 3D points.
%
% Output :
%   returns the original set of 3D points without the duplicates.
%   
%AUTHOR  : Neal Gauvin (at a3 dot epfl dot ch)
%VERSION : 1.0
%STATUS  : OK
%DATE    : 24 mai 2011

%Loop on all points to look for duplicates.
i1 = 1;
i2 = 2;
if( size(Points,1) == 3)
    while( i1 < size(Points,2) )
        while( i2 <= size(Points,2) )
            if( isequal(Points(:,i1),Points(:,i2) ) )
                Points(:,i2) = [];
                continue
            end
            i2=i2+1;
        end
        i1=i1+1;
        i2=i1+1;
    end
elseif( size(Points,2) == 3)
    while( i1 < size(Points,1) )
        while( i2 <= size(Points,1) )
            if( isequal(Points(i1,:),Points(i2,:) ) )
                Points(i2,:) = [];
                continue
            end
            i2=i2+1;
        end
        i1=i1+1;
        i2=i1+1;
    end
else
    display('Sorry, I can only work with 3xn or nx3 matrices of points')
    display('Size of the given matrix of points :')
    size(Points)
    
    
    
end


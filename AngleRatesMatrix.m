function [ E ] = AngleRatesMatrix( angles ) 
%creates rotation matrix using angles in radians
%angles = 3x1 array

    phi = angles(1);
    theta = angles(2);
    %psi = angles(3);
    
    E = zeros(3);

    E(1,1)= 1;
    E(1,3)= -sin(theta);

    E(2,2)= cos(phi);
    E(2,3)= cos(theta)*sin(phi);

    E(3,2)= -sin(phi);
    E(3,3)= cos(theta)*cos(phi);

end


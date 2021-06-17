function [ E_inv ] = InvAngleRatesMatrix( angles) 
%creates inverse angle rates matrix using euler angles in radians
%angles = 3x1 array

    phi = angles(1);
    theta = angles(2);
    %psi = angles(3);
    
    E_inv = zeros(3);

    E_inv(1,1)=cos(theta);
    E_inv(1,2)=sin(phi)*sin(theta);
    E_inv(1,3)=cos(phi)*sin(theta);

    E_inv(2,2)=cos(phi)*cos(theta);
    E_inv(2,3)=-sin(phi)*cos(theta);

    E_inv(3,2)= sin(phi);
    E_inv(3,3)= cos(phi);

    E_inv = (1/cos(theta))*E_inv;
end
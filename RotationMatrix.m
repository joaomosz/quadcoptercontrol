function [ R ] = RotationMatrix( angles )
%creates Euler rotation matrix using angles in radians
%angles = 3x1 array

    phi = angles(1);
    theta = angles(2);
    psi = angles(3);
    
    R = zeros(3);
    R(1,1)=cos(phi)*cos(psi);
    R(1,2)=cos(theta)*sin(psi);
    R(1,3)=-sin(theta);
    
    R(2,1)=sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi);
    R(2,2)=sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi);
    R(2,3)=cos(theta)*sin(phi);
    
    R(3,1)=cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    R(3,2)=cos(phi)*sin(theta)*sin(psi)-sin(phi)+cos(psi);
    R(3,3)=cos(theta)*cos(phi);
        

end


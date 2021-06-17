function [u_tilde] = equilibriumInput(X_tilde)
%returns the equilibrium input u_tilde related to the equilibrium state X_tilde
   
    m = 28e-3; %mass of crazifie (kg)
    kd = 0; %drag coefficient (kg/s) negligeble for low speeds
    g = 9.8; %earths gravity
    
    %equilibrium thrust
    eq =( m*g + kd*X_tilde(6) )/ cos(X_tilde(7))*cos(X_tilde(8));
    
    u_tilde = [0 0 0 eq].'; 
end


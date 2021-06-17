function [u_new] = normalizeU(u_old)
%normalizeU applies the phisical constraints of the input of the crazyfie
    u_new = zeros(size(u_old));
    u_new(1) = min(max(-0.01,u_old(1)),0.01);
    u_new(2) = min(max(-0.01,u_old(2)),0.01);
    u_new(3) = min(max(-0.02,u_old(3)),0.02);
    u_new(4) = min(max(0,u_old(4)),0.6388);
end


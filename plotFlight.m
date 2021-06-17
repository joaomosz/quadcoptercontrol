function [] = plotFlight(Results)
%plots the flight path in 3D
figure()
plot3(Results(1,:),Results(2,:),Results(3,:))
title('Flight path')
ylabel('Y')
xlabel('X')
zlabel('Z')
grid
end


function [] = plotStatesReduced(Results, t, name)
%plots the 6-dim state X (position and angles only) in a figure
figure('Name', name, 'Position', [10 10 400 800])
%positions
subplot(211)
hold on
plot(t,Results(1,:))
plot(t,Results(2,:))
plot(t,Results(3,:))
grid
title('Positions')
ylabel('Position (m)')
xlabel('time (s)')
legend('X', 'Y', 'Z')
hold off
%angles
subplot(212)
hold on
plot(t,Results(4,:)*180/pi)
plot(t,Results(5,:)*180/pi)
plot(t,Results(6,:)*180/pi)
grid
title('Angles')
ylabel('Angles (degrees)')
xlabel('time (s)')
legend('\phi', '\theta', '\psi')
hold off
end


function [] = plotStates(Results, t, name)
%plots the 12-dim state X in a figure
figure('Name', name, 'Position', [10 10 1000 800])
%positions
subplot(221)
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
%velocities
subplot(222)
hold on
plot(t,Results(4,:))
plot(t,Results(5,:))
plot(t,Results(6,:))
grid
title('Velocities')
ylabel('Velocity (m/s)')
xlabel('time (s)')
legend('v_x', 'v_y', 'v_z')
hold off
%angles
subplot(223)
hold on
plot(t,Results(7,:)*180/pi)
plot(t,Results(8,:)*180/pi)
plot(t,Results(9,:)*180/pi)
grid
title('Angles')
ylabel('Angles (degrees)')
xlabel('time (s)')
legend('\phi', '\theta', '\psi')
hold off
%angular velocities
subplot(224)
hold on
plot(t,Results(10,:))
plot(t,Results(11,:))
plot(t,Results(12,:))
grid
title(['Angular velocities'])
ylabel('Angular velocities (rad/s)')
xlabel('time (s)')
legend('\omega_\phi', '\omega_\theta', '\omega_\psi')
hold off

end


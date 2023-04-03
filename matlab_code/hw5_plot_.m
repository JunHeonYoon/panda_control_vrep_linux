function hw5_plot(data, time)
figure("Name","End-effector Position");
subplot(3, 1, 1)
plot(time, data(:, 4), "k--", "LineWidth", 3); hold on
plot(time, data(:, 1), "r"); hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 2)
plot(time, data(:, 5), "k--", "LineWidth", 3); hold on
plot(time, data(:, 2), "r"); hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 3)
plot(time, data(:, 6), "k--", "LineWidth", 3); hold on
plot(time, data(:, 3), "r"); hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])

% figure("Name","End-effector Linear Velocity");
% subplot(3, 1, 1)
% plot(time, data(:, 13), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 4), "r"); hold off
% title("v_X"); xlabel("Time[sec]"); ylabel("Velocity [m/s]"); grid on
% legend(["Desired", "Operated"])
% subplot(3, 1, 2)
% plot(time, data(:, 14), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 5), "r"); hold off
% title("v_Y"); xlabel("Time[sec]"); ylabel("Velocity [m/s]"); grid on
% legend(["Desired", "Operated"])
% subplot(3, 1, 3)
% plot(time, data(:, 15), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 6), "r"); hold off
% title("v_Z"); xlabel("Time[sec]"); ylabel("Velocity [m/s]"); grid on
% legend(["Desired", "Operated"])

% figure("Name","End-effector Angular velocity");
% subplot(3, 1, 1)
% plot(time, data(:, 16), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 7), "r"); hold off
% title("w_X"); xlabel("Time[sec]"); ylabel("Velocity [rad/s]"); grid on
% legend(["Desired", "Operated"])
% subplot(3, 1, 2)
% plot(time, data(:, 17), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 8), "r"); hold off
% title("w_Y"); xlabel("Time[sec]"); ylabel("Velocity [rad/s]"); grid on
% legend(["Desired", "Operated"])
% subplot(3, 1, 3)
% plot(time, data(:, 18), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 9), "r"); hold off
% title("w_Z"); xlabel("Time[sec]"); ylabel("Velocity [rad/s]"); grid on
% legend(["Desired", "Operated"])

figure("Name","End-effector Linear Velocity");
velocity = sqrt(data(:,7).^2 + data(:,8).^2 + data(:,9).^2);
plot(time, velocity)
% subplot(3, 1, 1)
% plot(time, data(:, 13), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 4), "r"); hold off
% title("v_X"); xlabel("Time[sec]"); ylabel("Velocity [m/s]"); grid on
% legend(["Desired", "Operated"])
% subplot(3, 1, 2)
% plot(time, data(:, 14), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 5), "r"); hold off
% title("v_Y"); xlabel("Time[sec]"); ylabel("Velocity [m/s]"); grid on
% legend(["Desired", "Operated"])
% subplot(3, 1, 3)
% plot(time, data(:, 15), "k--", "LineWidth", 3); hold on
% plot(time, data(:, 6), "r"); hold off
% title("v_Z"); xlabel("Time[sec]"); ylabel("Velocity [m/s]"); grid on
% legend(["Desired", "Operated"])

end
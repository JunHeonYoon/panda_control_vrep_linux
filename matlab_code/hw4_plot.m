function hw4_plot(data, time)

figure("Name","Joint Angle");
subplot(7,1,1)
plot(time, data(:, 1), "k--", "LineWidth", 3); hold on
plot(time, data(:, 8), "r"); hold off
title("q_1"); xlabel("Time [sec]"); ylabel("Angle [rad]"); grid on
legend(["Desired", "Operated"])
subplot(7,1,2)
plot(time, data(:, 2), "k--", "LineWidth", 3); hold on
plot(time, data(:, 9), "r"); hold off
title("q_2"); xlabel("Time [sec]"); ylabel("Angle [rad]"); grid on
legend(["Desired", "Operated"])
subplot(7,1,3)
plot(time, data(:, 3), "k--", "LineWidth", 3); hold on
plot(time, data(:, 10), "r"); hold off
title("q_3"); xlabel("Time [sec]"); ylabel("Angle [rad]"); grid on
legend(["Desired", "Operated"])
subplot(7,1,4)
plot(time, data(:, 4), "k--", "LineWidth", 3); hold on
plot(time, data(:, 11), "r"); hold off
title("q_4"); xlabel("Time [sec]"); ylabel("Angle [rad]"); grid on
legend(["Desired", "Operated"])
subplot(7,1,5)
plot(time, data(:, 5), "k--", "LineWidth", 3); hold on
plot(time, data(:, 12), "r"); hold off
title("q_5"); xlabel("Time [sec]"); ylabel("Angle [rad]"); grid on
legend(["Desired", "Operated"])
subplot(7,1,6)
plot(time, data(:, 6), "k--", "LineWidth", 3); hold on
plot(time, data(:, 13), "r"); hold off
title("q_6"); xlabel("Time [sec]"); ylabel("Angle [rad]"); grid on
legend(["Desired", "Operated"])
subplot(7,1,7)
plot(time, data(:, 7), "k--", "LineWidth", 3); hold on
plot(time, data(:, 14), "r"); hold off
title("q_7"); xlabel("Time [sec]"); ylabel("Angle [rad]"); grid on
legend(["Desired", "Operated"])

figure("Name","Applied Torque");
subplot(7,1,1)
plot(time, data(:,15), "r");
title("q_1"); xlabel("Time[sec]"); ylabel("Torque [Nm]"); grid on
subplot(7,1,2)
plot(time, data(:,16), "r");
title("q_2"); xlabel("Time[sec]"); ylabel("Torque [Nm]"); grid on
subplot(7,1,3)
plot(time, data(:,17), "r");
title("q_3"); xlabel("Time[sec]"); ylabel("Torque [Nm]"); grid on
subplot(7,1,4)
plot(time, data(:,18), "r");
title("q_4"); xlabel("Time[sec]"); ylabel("Torque [Nm]"); grid on
subplot(7,1,5)
plot(time, data(:,19), "r");
title("q_5"); xlabel("Time[sec]"); ylabel("Torque [Nm]"); grid on
subplot(7,1,6)
plot(time, data(:,20), "r");
title("q_6"); xlabel("Time[sec]"); ylabel("Torque [Nm]"); grid on
subplot(7,1,7)
plot(time, data(:,21), "r");
title("q_7"); xlabel("Time[sec]"); ylabel("Torque [Nm]"); grid on

end
function hw2_plot(data, time)

error = zeros(length(data), 1);
diff = zeros(length(data), 3);
diff = [ data(:,4)-data(:,1), data(:,5)-data(:,2), data(:,6)-data(:,3)];
for k=1:length(data)
    error(k, 1) = norm(diff(k));
end


figure("Name","End-effector Position");
subplot(4, 1, 1)
plot(time, data(:, 4), "k--", "LineWidth", 3); hold on
plot(time, data(:, 1), "r"); hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(4, 1, 2)
plot(time, data(:, 5), "k--", "LineWidth", 3); hold on
plot(time, data(:, 2), "r"); hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(4, 1, 3)
plot(time, data(:, 6), "k--", "LineWidth", 3); hold on
plot(time, data(:, 3), "r"); hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(4, 1, 4)
plot(time, error, "b-", "LineWidth", 3); 
title("Error"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on



figure("Name","Joint Angle");
subplot(7,1,1)
plot(time, data(:,7), "r");
title("q_1"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,2)
plot(time, data(:,8), "r");
title("q_2"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,3)
plot(time, data(:,9), "r");
title("q_3"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,4)
plot(time, data(:,10), "r");
title("q_4"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,5)
plot(time, data(:,11), "r");
title("q_5"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,6)
plot(time, data(:,12), "r");
title("q_6"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,7)
plot(time, data(:,13), "r");
title("q_7"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
end
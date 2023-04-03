function hw3_plot(data, time)

% error = zeros(length(data), 1);
% diff = zeros(length(data), 3);
% diff = [ data(:,4)-data(:,1), data(:,5)-data(:,2), data(:,6)-data(:,3)];
% for k=1:length(data)
%     error(k, 1) = norm(diff(k));
% end


figure("Name","End-effector Position");
subplot(3, 1, 1)
plot(time, data(:, 1), "k--", "LineWidth", 3); hold on
plot(time, data(:, 4), "r"); hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 2)
plot(time, data(:, 2), "k--", "LineWidth", 3); hold on
plot(time, data(:, 5), "r"); hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 3)
plot(time, data(:, 3), "k--", "LineWidth", 3); hold on
plot(time, data(:, 6), "r"); hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]");ylim([0.57,0.68]); grid on
legend(["Desired", "Operated"])
% subplot(4, 1, 4)
% plot(time, error, "b-", "LineWidth", 3); 
% title("Error"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on



figure("Name","End-effector Rotation");
subplot(3, 3, 1)
plot(time, data(:, 7), "k--", "LineWidth", 3); hold on
plot(time, data(:, 16), "r"); hold off
title("r{11}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{11})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])
subplot(3, 3, 4)
plot(time, data(:, 8), "k--", "LineWidth", 3); hold on
plot(time, data(:, 17), "r"); hold off
title("r{21}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{21})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])
subplot(3, 3, 7)
plot(time, data(:, 9), "k--", "LineWidth", 3); hold on
plot(time, data(:, 18), "r"); hold off
title("r{31}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{31})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])
subplot(3, 3, 2)
plot(time, data(:, 10), "k--", "LineWidth", 3); hold on
plot(time, data(:, 19), "r"); hold off
title("r{12}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{12})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])
subplot(3, 3, 5)
plot(time, data(:, 11), "k--", "LineWidth", 3); hold on
plot(time, data(:, 20), "r"); hold off
title("r{22}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{22})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])
subplot(3, 3, 8)
plot(time, data(:, 12), "k--", "LineWidth", 3); hold on
plot(time, data(:, 21), "r"); hold off
title("r{32}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{32})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])
subplot(3, 3, 3)
plot(time, data(:, 13), "k--", "LineWidth", 3); hold on
plot(time, data(:, 22), "r"); hold off
title("r{13}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{13})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])
subplot(3, 3, 6)
plot(time, data(:, 14), "k--", "LineWidth", 3); hold on
plot(time, data(:, 23), "r"); hold off
title("r{23}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{23})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])
subplot(3, 3, 9)
plot(time, data(:, 15), "k--", "LineWidth", 3); hold on
plot(time, data(:, 24), "r"); hold off
title("r{33}" ); xlabel("Time [sec]"); ylabel("Rotation Matrix Element(R_{33})");ylim([-1,1]); grid on
legend(["Desired", "Operated"])



figure("Name","Joint Angle");
subplot(7,1,1)
plot(time, data(:,25), "r");
title("q_1"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,2)
plot(time, data(:,26), "r");
title("q_2"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,3)
plot(time, data(:,27), "r");
title("q_3"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,4)
plot(time, data(:,28), "r");
title("q_4"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,5)
plot(time, data(:,29), "r");
title("q_5"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,6)
plot(time, data(:,30), "r");
title("q_6"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,7)
plot(time, data(:,31), "r");
title("q_7"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
end
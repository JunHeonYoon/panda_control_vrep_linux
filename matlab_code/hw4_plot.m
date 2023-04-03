function hw4_plot(data, time)

% error_ee = zeros(length(data), 1);
% diff_ee = zeros(length(data), 3);
% diff_ee = [ data(:,7)-data(:,1), data(:,8)-data(:,2), data(:,9)-data(:,3)];
% for k=1:length(data)
%     error_ee(k, 1) = norm(diff_ee(k));
% end
% 
% error_com = zeros(length(data), 1);
% diff_com = zeros(length(data), 3);
% diff_com = [ data(:,10)-data(:,4), data(:,11)-data(:,5), data(:,12)-data(:,6)];
% for k=1:length(data)
%     error_com(k, 1) = norm(diff_com(k));
% end


figure("Name","End-effector Position");
subplot(3, 1, 1)
plot(time, data(:, 1), "k--", "LineWidth", 3); hold on
plot(time, data(:, 7), "r"); hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 2)
plot(time, data(:, 2), "k--", "LineWidth", 3); hold on
plot(time, data(:, 8), "r"); hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 3)
plot(time, data(:, 3), "k--", "LineWidth", 3); hold on
plot(time, data(:, 9), "r"); hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]");ylim([0.64, 0.66]); grid on
legend(["Desired", "Operated"])
% subplot(4, 1, 4)
% plot(time, error_ee, "b-", "LineWidth", 3); 
% title("Error"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on

figure("Name","COM of Link 4 Position");
subplot(3, 1, 1)
plot(time, data(:, 4), "k--", "LineWidth", 3); hold on
plot(time, data(:, 10), "r"); hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 2)
plot(time, data(:, 5), "k--", "LineWidth", 3); hold on
plot(time, data(:, 11), "r"); hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 3)
plot(time, data(:, 6), "k--", "LineWidth", 3); hold on
plot(time, data(:, 12), "r"); hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
% subplot(4, 1, 4)
% plot(time, error_com, "b-", "LineWidth", 3); 
% title("Error"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on


figure("Name","Joint Angle");
subplot(7,1,1)
plot(time, data(:,13), "r");
title("q_1"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,2)
plot(time, data(:,14), "r");
title("q_2"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,3)
plot(time, data(:,15), "r");
title("q_3"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,4)
plot(time, data(:,16), "r");
title("q_4"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,5)
plot(time, data(:,17), "r");
title("q_5"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,6)
plot(time, data(:,18), "r");
title("q_6"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
subplot(7,1,7)
plot(time, data(:,19), "r");
title("q_7"); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on
end
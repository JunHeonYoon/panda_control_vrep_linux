function hw6_plot(data_index, data)

Name = ["HW 6-1", "HW 6-2"];
time = (1:length(data{1}))*0.01;
Color = [0, 0.4470, 0.7410; 
         0.8500, 0.3250, 0.0980];


f1 = figure("Name","End-effector Position");
f1.WindowState = 'maximized';
subplot(4, 1, 1)
plot(time, data{1}(:, 1), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 7), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(4, 1, 2)
plot(time, data{1}(:, 2), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 8), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(4, 1, 3)
plot(time, data{1}(:, 3), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 9), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(4, 1, 4)
for i=data_index
    plot(time, data{i}(:, 13), "Color", Color(i,:),"LineWidth", 2); hold on
end
hold off
title("h_1"); xlabel("Time[sec]"); ylabel("Value"); grid on
pause(1)
saveas(gcf, strcat("plot figure/HW6/HW_", erase(num2str(data_index)," "), "(posi).svg"))



f2 = figure("Name","COM of Link 4");
f2.WindowState = 'maximized';
subplot(4, 1, 1)
plot(time, data{1}(:, 4), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 10), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(4, 1, 2)
plot(time, data{1}(:, 5), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 11), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(4, 1, 3)
plot(time, data{1}(:, 6), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 12), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(4, 1, 4)
for i=data_index
    plot(time, data{i}(:, 13), "Color", Color(i,:),"LineWidth", 2); hold on
end
hold off
title("h_1"); xlabel("Time[sec]"); ylabel("Value"); grid on
pause(1)
saveas(gcf, strcat("plot figure/HW6/HW_", erase(num2str(data_index)," "), "-lin4(posi).svg"))


f3 = figure("Name","Joint Angle");
f3.WindowState = 'maximized';
for i=1:7
    subplot(7,1,i)
    hold on
    for j=data_index
        plot(time, data{j}(:,13+i), "Color", Color(j,:), "LineWidth", 2);
    end
    hold off
    title(strcat("q_", num2str(i))); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on; legend(Name(data_index), 'Location','northeastoutside')
end
pause(1)
saveas(gcf, strcat("plot figure/HW6/HW_", erase(num2str(data_index)," "), "(joint).svg"))

end
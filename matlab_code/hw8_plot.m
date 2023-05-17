function hw8_plot(data_index, data)

Name = ["HW 8-1-1", "HW 8-1-2", "HW 8-2-1", "HW 8-2-2"];
time = (1:length(data{data_index(1)}))*0.001;
Color = [0, 0.4470, 0.7410; 
         0.8500, 0.3250, 0.0980;
         0.4940, 0.1840, 0.5560;
         0.4660, 0.6740, 0.1880];
tmp_index = [7 10 13 8 11 14 9 12 15];


f1 = figure("Name","End-effector Position");
f1.WindowState = 'maximized';
subplot(3, 1, 1)
plot(time, data{data_index(1)}(:, 1), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 4), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(3, 1, 2)
plot(time, data{data_index(1)}(:, 2), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 5), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(3, 1, 3)
plot(time, data{data_index(1)}(:, 3), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 6), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]");ylim([0.57,0.68]); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
pause(1)
saveas(gcf, strcat("plot figure/HW8/HW_", erase(num2str(data_index)," "), "(posi).svg"))



f2 = figure("Name","End-effector Rotation");
f2.WindowState = 'maximized';
for i=1:9
    subplot(3,3,i)
    plot(time, data{data_index(1)}(:, tmp_index(i)), "k--", "LineWidth", 3); hold on
    for j=data_index
        plot(time, data{j}(:, tmp_index(i)+9), "Color", Color(j,:),"LineWidth", 2); 
    end
    hold off
    title(strcat('r_{', num2str(fix((i-1)/3)+1), num2str(rem(i-1,3)+1), '}')); 
    xlabel("Time [sec]"); ylabel(strcat('Rotation Matrix Element(R_{', num2str(fix((i-1)/3)+1), num2str(rem(i-1,3)+1), '})')  );ylim([-1,1]); grid on
    legend(["Desired", Name(data_index)],'Location','northeastoutside');
end
pause(1)
saveas(gcf, strcat("plot figure/HW8/HW_", erase(num2str(data_index)," "), "(ori).svg"))


f3 = figure("Name","Joint Angle");
f3.WindowState = 'maximized';
for i=1:7
    subplot(7,1,i)
    hold on
    for j=data_index
        plot(time, data{j}(:,24+i), "Color", Color(j,:), "LineWidth", 2);
    end
    hold off
    title(strcat("q_", num2str(i))); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on; legend(Name(data_index), 'Location','northeastoutside')
end
pause(1)
saveas(gcf, strcat("plot figure/HW8/HW_", erase(num2str(data_index)," "), "(joint).svg"))

end
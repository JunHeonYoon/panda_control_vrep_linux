function hw7_plot(data_index, data)

Name = ["HW 7-1", "HW 7-2-1", "HW 7-2-2", "HW 7-3-1", "HW 7-3-2"];
time = (1:length(data{data_index(1)}))*0.001;
Color = [0, 0.4470, 0.7410; 
         0.8500, 0.3250, 0.0980;
         0.4940, 0.1840, 0.5560;
         0.4660, 0.6740, 0.1880
         0.6350 0.0780 0.1840];

box_data = cell(size(data));
for i=1:length(box_data)
    for j=1:7
        box_data{i}(1,j) = mean(data{i}(:,7+j)) - min(data{i}(:,7+j));
        box_data{i}(2,j) = max(data{i}(:,7+j)) - mean(data{i}(:,7+j));
        box_data{i}(3,j) = mean(data{i}(:,7+j));
    end
end

f1 = figure("Name","Joint Angle");
f1.WindowState = 'maximized';
for i=1:7
    subplot(7,1,i)
    plot(time, data{data_index(1)}(:,i), "k-- ","LineWidth", 3); hold on
    for j=data_index
        plot(time, data{j}(:,7+i), "Color",Color(j,:), "LineWidth",2)
        xlim tight; ylim([box_data{j}(3,i) - max(box_data{j}(1,:)), box_data{j}(3,i) + max(box_data{j}(2,:))])
    end
    hold off
    title(strcat("q_", num2str(i))); xlabel("Time[sec]"); ylabel("Angle [rad]"); grid on; legend(["desired", Name(data_index)], 'Location','northeastoutside')
end
pause(1)
saveas(gcf, strcat("plot figure/HW7/HW_", erase(num2str(data_index)," "), "(joint).svg"))

end
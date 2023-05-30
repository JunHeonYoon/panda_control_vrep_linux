function hw10_plot(data_index, data)

Name = ["HW 10-1-1", "HW 10-1-2", "HW 10-2", "HW 10-3"];
time = (1:length(data{data_index(1)}))*0.001;
Color = [0, 0.4470, 0.7410; 
         0.8500, 0.3250, 0.0980;
         0.4940, 0.1840, 0.5560;
         0.4660, 0.6740, 0.1880];
tmp_index = [7 10 13 8 11 14 9 12 15];

box_data = cell(size(data));
for i=1:length(box_data)
    for j=1:3
        box_data{i}(1,j) = mean(data{i}(:,j)) - min(data{i}(:,j));
        box_data{i}(2,j) = max(data{i}(:,j)) - mean(data{i}(:,j));
        box_data{i}(3,j) = mean(data{i}(:,j));
    end
end


f1 = figure("Name","End-effector Position");
f1.WindowState = 'maximized';
subplot(3, 1, 1)
plot(time, data{data_index(1)}(:, 1), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 4), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
xlim tight; ylim([box_data{data_index(1)}(3,1) - max(box_data{data_index(1)}(1,:)), box_data{data_index(1)}(3,1) + max(box_data{data_index(1)}(2,:))])
title("X"); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(3, 1, 2)
plot(time, data{data_index(1)}(:, 2), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 5), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
xlim tight; ylim([box_data{data_index(1)}(3,2) - max(box_data{data_index(1)}(1,:)), box_data{data_index(1)}(3,2) + max(box_data{data_index(1)}(2,:))])
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
subplot(3, 1, 3)
plot(time, data{data_index(1)}(:, 3), "k--", "LineWidth", 3); hold on
for i=data_index
    plot(time, data{i}(:, 6), "Color", Color(i,:),"LineWidth", 2); 
end
hold off
xlim tight; ylim([box_data{data_index(1)}(3,3) - max(box_data{data_index(1)}(1,:)), box_data{data_index(1)}(3,3) + max(box_data{data_index(1)}(2,:))])
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", Name(data_index)], 'Location','northeastoutside')
pause(1)
saveas(gcf, strcat("plot figure/HW10/HW_", erase(num2str(data_index)," "), "(posi).svg"))



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
saveas(gcf, strcat("plot figure/HW10/HW_", erase(num2str(data_index)," "), "(ori).svg"))


if ismember(1, data_index>2)
    f3 = figure("Name","End-effector Linear Velocity");
    f3.WindowState = 'maximized';
    plot(time, 0.3*ones(length(time),1), "r--", "LineWidth",3); hold on
    for i=data_index(data_index > 2)
        velocity = zeros(length(data{i}),2);
        for j=1:length(velocity)
            velocity(j,1) = norm(data{i}(j,25:27));
            velocity(j,2) = norm(data{i}(j,28:30));
        end
        plot(time, velocity(:,1), "k--","LineWidth",3); 
        plot(time, velocity(:,2), "Color", Color(i,:),"LineWidth",2); 
    end
    hold off;
    title("Linear Velocity"); 
    xlabel("Time [sec]"); ylabel("Velocity [m/s]"); grid on
    legend(["Max", "Desired", Name(data_index(data_index > 2))],'Location','northeastoutside');
    pause(1)
    saveas(gcf, strcat("plot figure/HW10/HW_", erase(num2str(data_index)," "), "(vel).svg"))
end

if ismember(4,data_index)
    f4 = figure("Name","End-effector Position with obstacle");
    f4.WindowState = 'maximized';
    load('data/mesh_light.mat');
    mesh{8}.v = mesh{8}.v *[1 0 0;0 -1 0;0 0 -1];
    plot3(data{4}(:,1),data{4}(:,2),data{4}(:,3),"k--","LineWidth",3); hold on
    plot3(data{4}(:,4),data{4}(:,5),data{4}(:,6),"Color", Color(4,:),"LineWidth",2);
    [X,Y,Z] = sphere;
    X=X*0.1+0.15;Y=Y*0.1-0.012;Z=Z*0.1+0.65;
    surf(X,Y,Z)
    for j=0:15    
        patch('Faces',mesh{8}.f,'Vertices', mesh{8}.v+repmat(data{4}(j*500+1,4:6)+[0,0,0.105],[length(mesh{8}.v),1]) ...
            ,'facecolor',[1 0 0],'Facealpha',0.05,'edgealpha',0)
        % patch('Faces',mesh{8}.f,'Vertices', mesh{8}.v+repmat([0,0,0.105],[length(mesh{8}.v),1]) ...
        %     ,'facecolor',[1 0 0],'Facealpha',0.05,'edgealpha',0)
    end
    hold off
    xlabel("X","FontWeight","bold"); ylabel("Y","FontWeight","bold"); zlabel("Z","FontWeight","bold")
    axis equal
    view(0,0)
    pause(1)
    saveas(gcf, strcat("plot figure/HW10/HW_", erase(num2str(data_index)," "), "(task space).svg"))
end

end
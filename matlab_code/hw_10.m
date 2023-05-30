clc; close all; clear all;

hw_10_1_1 = true;
hw_10_1_2 = true;
hw_10_2 = true;
hw_10_3 = true;



data = cell(1,4);
for i=1:3
    if i==1
        for j=1:2
            data{j} = importdata(strcat("data/hw_10_", num2str(i),"_", num2str(j), ".txt"), ' ', 0);
            data{j} = data{j}(1:4000, :);
        end
    else
        data{i+1} = importdata(strcat("data/hw_10_", num2str(i), ".txt"), ' ', 0);
        if i==3
            data{i+1} = data{i+1}(1:8000, :);
        else
            data{i+1} = data{i+1}(1:4000, :);
        end
    end
end

if hw_10_1_1
    hw10_plot([1], data)
end
if hw_10_1_2
    hw10_plot([2], data)
end
if hw_10_2
    hw10_plot([3], data)
end
if hw_10_3
    hw10_plot([4], data)
end
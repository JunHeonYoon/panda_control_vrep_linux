clc; close all; clear all;

hw_5_1 = true;
hw_5_2 = true;

hw_5_12 = true;


data = cell(1,2);
for i=1:2
    data{i} = importdata(strcat("data/hw_5_", num2str(i), ".txt"), ' ', 0);
    data{i} = data{i}(1:800, :);
end

if hw_5_1
    hw5_plot([1], data)
end
if hw_5_2
    hw5_plot([2], data)
end
if hw_5_12
    hw5_plot([1,2],data)
end

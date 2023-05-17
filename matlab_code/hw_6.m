clc; close all; clear all;

hw_6_1 = true;
hw_6_2 = true;

hw_6_12 = true;


data = cell(1,2);
for i=1:2
    data{i} = importdata(strcat("data/hw_6_", num2str(i), ".txt"), ' ', 0);
    data{i} = data{i}(1:800, :);
end

if hw_6_1
    hw6_plot([1], data)
end
if hw_6_2
    hw6_plot([2], data)
end
if hw_6_12
    hw6_plot([1,2],data)
end

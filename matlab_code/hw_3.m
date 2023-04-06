clc; close all; clear all;

hw_3_1 = false;
hw_3_2 = true;
hw_3_3 = false;
hw_3_4 = false;

hw_3_123 = true;
hw_3_234 = true;


data = cell(1,4);
for i=1:4
    data{i} = importdata(strcat("data/hw_3_", num2str(i), ".txt"), ' ', 0);
    data{i} = data{i}(1:400, :);
end

if hw_3_1
    hw3_plot([1], data)
end
if hw_3_2
    hw3_plot([2], data)
end
if hw_3_3
    hw3_plot([3], data)
end
if hw_3_4
    hw3_plot([4], data)
end
if hw_3_123
    hw3_plot([1,2,3],data)
end
if hw_3_234
    hw3_plot([2,3,4], data)
end
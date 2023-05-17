clc; close all; clear all;

hw_7_1 = true;
hw_7_2_1 = true;
hw_7_2_2 = true;
hw_7_3_1 = true;
hw_7_3_2 = true;



data = cell(1,5);
for i=1:3
    if i == 1
        data{i} = importdata(strcat("data/hw_7_", num2str(i), ".txt"), ' ', 0);
        data{i} = data{i}(1:4000, :);
    else
        for j=1:2
            data{(i-1)*2+(j-1)} = importdata(strcat("data/hw_7_", num2str(i), "_", num2str(j), ".txt"), ' ', 0);
            data{(i-1)*2+(j-1)} = data{(i-1)*2+(j-1)}(1:4000, :);
        end
    end
end

if hw_7_1
    hw7_plot([1], data)
end
if hw_7_2_1
    hw7_plot([2], data)
end
if hw_7_2_2
    hw7_plot([3], data)
end
if hw_7_3_1
    hw7_plot([4], data)
end
if hw_7_3_2
    hw7_plot([5], data)
end
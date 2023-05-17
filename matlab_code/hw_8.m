clc; close all; clear all;

hw_8_1_1 = true;
hw_8_1_2 = true;
hw_8_2_1 = true;
hw_8_2_2 = true;

hw_8__1 = true;
hw_8__2 = true;


data = cell(1,4);
for i=1:2
    for j=1:2
        data{(i-1)*2+j} = importdata(strcat("data/hw_8_", num2str(i),"_", num2str(j), ".txt"), ' ', 0);
        data{(i-1)*2+j} = data{(i-1)*2+j}(1:4000, :);
    end
end

if hw_8_1_1
    hw8_plot([1], data)
end
if hw_8_1_2
    hw8_plot([2], data)
end
if hw_8_2_1
    hw8_plot([3], data)
end
if hw_8_2_2
    hw8_plot([4], data)
end
if hw_8__1
    hw8_plot([1,3], data)
end
if hw_8__2
    hw8_plot([2,4], data)
end

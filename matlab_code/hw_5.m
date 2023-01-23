clc; close all; clear all;

hw_5_1_1 = false;
hw_5_1_2 = false;
hw_5_2 = true;


file_name1_1 = 'data/hw_5_1_1.txt';
file_name1_2 = 'data/hw_5_1_2.txt';
file_name2 = 'data/hw_5_2.txt';
delimiterIn = ' ';
headerlineIn = 0;


if hw_5_1_1
    data1 = importdata(file_name1_1, delimiterIn, headerlineIn);
    data1 = data1(1:4000, :);
    t1_1 = length(data1);
    time1_1 = 0.001*(1:t1_1);
    hw5_plot(data1, time1_1)
end
if hw_5_1_2
    data1_2 = importdata(file_name1_2, delimiterIn, headerlineIn);
    data1_2 = data1_2(1:4000, :);
    t1_2 = length(data1_2);
    time1_2 = 0.001*(1:t1_2);
    hw5_plot(data1_2, time1_2)
end
if hw_5_2
    data2 = importdata(file_name2, delimiterIn, headerlineIn);
    data2 = data2(1:4000, :);
    t2 = length(data2);
    time2 = 0.001*(1:t2);
    hw5_plot(data2, time2)
end


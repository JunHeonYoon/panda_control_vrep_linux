clc; close all; clear all;

hw_5_1 = false;
hw_5_2_1 = false;
hw_5_2_2 = false;
hw_5_3_1 = true;
hw_5_3_2 = true;


file_name1 = 'data/hw_5_1.txt';
file_name2_1 = 'data/hw_5_2_1.txt';
file_name2_2 = 'data/hw_5_2_2.txt';
file_name3_1 = 'data/hw_5_3_1.txt';
file_name3_2 = 'data/hw_5_3_2.txt';
delimiterIn = ' ';
headerlineIn = 0;


if hw_5_1
    data1 = importdata(file_name1, delimiterIn, headerlineIn);
    data1 = data1(1:4000, :);
    t1 = length(data1);
    time1 = 0.001*(1:t1);
    hw5_plot(data1, time1)
end
if hw_5_2_1
    data2_1 = importdata(file_name2_1, delimiterIn, headerlineIn);
    data2_1 = data2_1(1:4000, :);
    t2_1 = length(data2_1);
    time2_1 = 0.001*(1:t2_1);
    hw5_plot(data2_1, time2_1)
end
if hw_5_2_2
    data2_2 = importdata(file_name2_2, delimiterIn, headerlineIn);
    data2_2 = data2_2(1:4000, :);
    t2_2 = length(data2_2);
    time2_2 = 0.001*(1:t2_2);
    hw5_plot(data2_2, time2_2)
end
if hw_5_3_1
    data3_1 = importdata(file_name3_1, delimiterIn, headerlineIn);
    data3_1 = data3_1(1:4000, :);
    t3 = length(data3_1);
    time3 = 0.001*(1:t3);
    hw5_plot(data3_1, time3)
end
if hw_5_3_2
    data3_2 = importdata(file_name3_2, delimiterIn, headerlineIn);
    data3_2 = data3_2(1:4000, :);
    t4 = length(data3_2);
    time4 = 0.001*(1:t4);
    hw5_plot(data3_2, time4)
end


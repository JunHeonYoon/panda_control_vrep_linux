clc; close all; clear all;

hw_2_1 = true;
hw_2_2 = true;
hw_2_3 = true;


file_name1 = 'data/hw_2_1.txt';
file_name2 = 'data/hw_2_2.txt';
file_name3 = 'data/hw_2_3.txt';
delimiterIn = ' ';
headerlineIn = 0;


if hw_2_1
    data1 = importdata(file_name1, delimiterIn, headerlineIn);
    data1 = data1(1:200, :);
    t1 = length(data1);
    time1 = 0.01*(1:t1);
    hw2_plot(data1, time1)
end
if hw_2_2
    data2 = importdata(file_name2, delimiterIn, headerlineIn);
    data2 = data2(1:200, :);
    t2 = length(data2);
    time2 = 0.01*(1:t2);
    hw2_plot(data2, time2)
end
if hw_2_3
    data3 = importdata(file_name3, delimiterIn, headerlineIn);
    data3 = data3(1:200, :);
    t3 = length(data3);
    time3 = 0.01*(1:t3);
    hw2_plot(data3, time3)
    
end

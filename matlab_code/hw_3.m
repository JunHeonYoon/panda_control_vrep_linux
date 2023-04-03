clc; close all; clear all;

hw_3_1 = true;
hw_3_2 = true;
hw_3_3 = true;
hw_3_4 = true;


file_name1 = 'data/hw_3_1.txt';
file_name2 = 'data/hw_3_2.txt';
file_name3 = 'data/hw_3_3.txt';
file_name4 = 'data/hw_3_4.txt';
delimiterIn = ' ';
headerlineIn = 0;


if hw_3_1
    data1 = importdata(file_name1, delimiterIn, headerlineIn);
    data1 = data1(1:400, :);    
    t1 = length(data1);
    time1 = 0.01*(1:t1);
    hw3_plot(data1, time1)
end
if hw_3_2
    data2 = importdata(file_name2, delimiterIn, headerlineIn);
    data2 = data2(1:400, :);
    t2 = length(data2);
    time2 = 0.01*(1:t2);
    hw3_plot(data2, time2)
end
if hw_3_3
    data3 = importdata(file_name3, delimiterIn, headerlineIn);
    data3 = data3(1:400, :);
    t3 = length(data3);
    time3 = 0.01*(1:t3);
    hw3_plot(data3, time3)
end
if hw_3_4
    data4 = importdata(file_name4, delimiterIn, headerlineIn);
    data4 = data4(1:400, :);    
    t1 = length(data4);
    time1 = 0.01*(1:t1);
    hw3_plot(data4, time1)
end

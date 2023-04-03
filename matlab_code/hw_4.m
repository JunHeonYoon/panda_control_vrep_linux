clc; close all; clear all;

hw_4_1 = true;
hw_4_2 = true;



file_name1 = 'data/hw_4_1.txt';
file_name2 = 'data/hw_4_2.txt';
delimiterIn = ' ';
headerlineIn = 0;


if hw_4_1
    data1 = importdata(file_name1, delimiterIn, headerlineIn);
    data1 = data1(1:400, :);    
    t1 = length(data1);
    time1 = 0.01*(1:t1);
    hw4_plot(data1, time1)
end
if hw_4_2
    data2 = importdata(file_name2, delimiterIn, headerlineIn);
    data2 = data2(1:400, :);
    t2 = length(data2);
    time2 = 0.01*(1:t2);
    hw4_plot(data2, time2)
end

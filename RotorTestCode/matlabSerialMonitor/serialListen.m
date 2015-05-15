close all; clear all; clc; 
s = serial('COM5');%Set litening comm port
set(s, 'BaudRate', 9600);
fopen(s);
fprintf(s, '*IDN?');
out = fscanf(s)
fclose(s)
delete(s)
clear s
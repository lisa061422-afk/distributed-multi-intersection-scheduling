% This is the main program to generate contention nodes with 10 columns for
% each node

disp('Running Main_A_star...');
Main; % 运行子程序1

disp('Update h true cost...');
Update_true_h_cost; % 运行子程序2

disp('Extract contention nodes...');
Extract_ContentionNodes; % 运行子程序3

disp('Remove columns 8,11,12,13...');
remove_cols; % 运行子程序4

disp('Produce excel file...');
Excel_Produce; % 运行子程序5


disp('All subprograms have been executed.');
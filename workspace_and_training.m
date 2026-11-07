clc; clear all; close all;

%===============================
% 10x10 checks for 100 examples
%===============================
alpha1 = linspace(0.01, pi/4, 10);
alpha2 = -linspace(0.01, pi/4, 10);
k=1;

%===============================
% inputs 2xN of angles(alphas)
% outputs 2xN of end effector points
% (x;y)=(E(1);E(2))
%===============================
for i = 1:length(alpha1)
    for j = 1:length(alpha2)
        out = Forward_Kin([alpha1(i), alpha2(j)]);
        inputs(:,k) = out(1:2);
        outputs(:,k) = out(3:4);
        k=k+1;
    end
end
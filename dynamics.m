%% Housekeeping
close all
clear
clc

%% 2nd Order Unicle Dynamics
A = [0,0,1,0;
     0,0,0,1;
     0,0,0,0;
     0,0,0,0];
B = [0,0;
     0,0;
     1,0;
     0,1];

syms t u1 u2
stm = expm(A*t);

integrand = stm*B*[u1;u2];

rhs = int(integrand, t);

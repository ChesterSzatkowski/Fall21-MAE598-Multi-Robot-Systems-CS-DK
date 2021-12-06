%% MAE 598 Final Project
% Daniel Kim , Chester Szatkowski
% Fall 2021

%% Clearing
clear all;
clc;
%% Numerical Solving
species = 7;
tspan = [0 1000]; %seconds
x0 = [0.33, % initial robot population fraction
    0.33, % initial small object population fraction
    0.33, % initial large object population fraction
    0, % initial large transportation population fraction
    0, % initial small transportation population fraction
    0, % initial delivered large object population fraction
    0]; % initial delivered small object population fraction

M = readmatrix('M.txt');

alpha1 = 0.02; % rate for robots working to move a large object into transportation phase
alpha2 = 0.01; % rate for robot working to move a small object into transportation phase
alpha3 = 0.01; % rate for large object to be delivered and 4 robots to be freed
alpha4 = 0.01; % rate for small object to be delivered and a robot to be freed
alpha5 = 0.003; % rate for a robot to take 10 small objects to make a large object
beta = 0.001; % rate for robot to disassemble a large object into 10 small objects

K = [alpha1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    -alpha1, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    0, 0, alpha2, 0, 0, 0, 0, 0, 0, 0;
    0, 0, -alpha2, 0, 0, 0, 0, 0, 0, 0;
    0, 0, 0, 0, alpha3, 0, 0, 0, 0, 0;
    0, 0, 0, 0, -alpha3, 0, 0, 0, 0, 0;
    0, 0, 0, 0, 0, 0, alpha4, 0, 0, 0;
    0, 0, 0, 0, 0, 0, -alpha4, 0, 0, 0;
    0, 0, 0, 0, 0, 0, 0, 0, alpha5, -beta;
    0, 0, 0, 0, 0, 0, 0, 0, -alpha5, beta];

[t,x] = ode45(@(t,x) odefun(t,x,M,K), tspan, x0);
hold on;
plot(t,x(:,1),'r'); %robots
plot(t,x(:,2),'g'); % small objects
plot(t,x(:,3),'b'); % large objects
plot(t,x(:,4),'m'); % transporting large objects
plot(t,x(:,5),'c'); % transporting small objects
plot(t,x(:,6),'y'); % delivered large object
plot(t,x(:,7),'k'); % delivered small object
legend('Robots', 'Small Objects', 'Large Objects', 'Large Transportation', 'Small Transportation', 'Delivered Large', 'Delivered Small')

function V = gma(D,k,L)
%
% Supplementary Matlab files for the paper:
%
% " Deterministic Modelling and Stochastic Simulation of Pathways using Matlab"
% by M.Ullah, H.Schmidt, K-H.Cho and O.Wolkenhauer
%
% This function implements the generalized law of mass action
%
% We would appreciate a citation if these files are used by others.

if nargin < 3
    L = -D.*(D < 0);
    i2 = L > 1;
else
    i2 = L>0 & L~=1 ;
end
i0 = L==0;
M1s = ones(size(k));
V = @Vfn;
    function xd = Vfn(t,x)          
        X = x(:,M1s);
        X(i0) = 1;
        X(i2) = X(i2).^L(i2);        
        xd = D*(k.*prod(X)).';
    end
end
function dxdt = odefun(t,x,M,K)
% x = [x(1); x(2); x(3); x(4); x(5); x(6); x(7)];
y = [4*x(1)*x(3); x(4); x(1)*x(2); x(5); x(4); 4*x(1)*x(6); x(5); x(7)*x(1); 10*x(2)*x(1); x(3)*x(1)];

dxdt = -M*K*y;
end



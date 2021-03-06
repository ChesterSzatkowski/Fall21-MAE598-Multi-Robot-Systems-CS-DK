%% MAE 598 Final Project
% Daniel Kim , Chester Szatkowski
% Fall 2021

%% Clearing
clear all;
clc;
%% ODE Numerical Solving
species = 7;
tspan = [0 3600]; % seconds
x0 = [0.05, % initial robot population fraction
    0.3, % initial small object population fraction
    0.6, % initial large object population fraction
    0, % initial large transportation population fraction
    0, % initial small transportation population fraction
    0, % initial delivered large object population fraction
    0]; % initial delivered small object population fraction

Ntot = 1000;

M = [4 0 1 0 0 4 0 1 1 1;
    0 0 1 0 0 0 0 0 10 0;
    1 0 0 0 0 0 0 0 0 1;
    0 1 0 0 1 0 0 0 0 0;
    0 0 0 1 0 0 1 0 0 0;
    0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0];


alpha1 = 0.08; % rate for robots working to move a large object into transportation phase
alpha2 = 0.064; % rate for robot working to move a small object into transportation phase
alpha3 = 0.032; % rate for large object to be delivered and 4 robots to be freed
alpha4 = 0.016; % rate for small object to be delivered and a robot to be freed
alpha5 = 0.0032; % rate for a robot to take 10 small objects to make a large object
beta = 0.00144; % rate for robot to disassemble a large object into 10 small objects

k = [alpha1, alpha2, alpha3, alpha4, alpha5, beta];
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
figure(1)
hold on
set(gca,'Fontsize',20);
grid on
plot(t,Ntot*x(:,1),'r'); %robots
plot(t,Ntot*x(:,2),'g'); % small objects
plot(t,Ntot*x(:,3),'b'); % large objects
plot(t,Ntot*x(:,4),'m'); % transporting large objects
plot(t,Ntot*x(:,5),'c'); % transporting small objects
plot(t,Ntot*x(:,6),'y'); % delivered large object
plot(t,Ntot*x(:,7),'k'); % delivered small object


%% Gillespie

m12 = M(:,2) - M(:,1); % reaction 1
m34 = M(:,4) - M(:,3); % reaction 2
m56 = M(:,6) - M(:,5); % reaction 3
m78 = M(:,8) - M(:,7); % reaction 4
m910 = M(:,10) - M(:,9); % reaction 5
m109 = -m910; % reaction 6

%Reaction vectors
R = [m12 m34 m56 m78 m910 m109]';

c1 = alpha1/Ntot; % reaction 1
c2 = alpha2/Ntot; % reaction 2
c3 = alpha3; % reaction 3
c4 = alpha4; % reaction 4
c5 = alpha5/Ntot; % reaction 5
c6 = beta/Ntot; % reaction 6


t = 0;
tfinal = 3600;
N = Ntot*x0';
tvec = t;
Nvec = N/Ntot;

while t < tfinal
    
    % Reaction propensities
    a(1) = c1*N(1)*N(3); % reaction 1
    a(2) = c2*N(1)*N(2); % reaction 2
    a(3) = c3*N(4); % reaction 3
    a(4) = c4*N(5); % reaction 4
    a(5) = c5*N(2)*N(1); % reaction 5
    a(6)= c6*N(3)*N(1);  % reaction 6
     
    asum = sum(a);
    j = min(find(rand<cumsum(a/asum))); % index of the next reaction
    tau = log(1/rand)/asum;
    N = N + R(j,:); % simulate the reaction occurring;
                    % update the integer population counts
    t = t + tau; % time of the next reaction
    tvec(end+1) = t;
    Nvec(end+1,:) = N/Ntot;
end

plot(tvec,Ntot*Nvec(:,1),'r','LineStyle','--'); %robots
plot(tvec,Ntot*Nvec(:,2),'g','LineStyle','--'); % small objects
plot(tvec,Ntot*Nvec(:,3),'b','LineStyle','--'); % large objects
plot(tvec,Ntot*Nvec(:,4),'m','LineStyle','--'); % transporting large objects
plot(tvec,Ntot*Nvec(:,5),'c','LineStyle','--'); % transporting small objects
plot(tvec,Ntot*Nvec(:,6),'y','LineStyle','--'); % delivered large object
plot(tvec,Ntot*Nvec(:,7),'k','LineStyle','--'); % delivered small object
xlabel('Time (s)');
ylabel('Quantity');
xlim([0 tfinal*1.2])
set(gcf, 'Position',  [100, 100, 1800, 900])

legend('Robots', 'Small Objects', 'Large Objects', 'Large Transportation', 'Small Transportation', 'Delivered Large', 'Delivered Small');
%xlim([0 tfinal])
%}

%% Comparing weakly-reversible system

M2 = [4 0 1 0 0 4 0 1 1 1;
    0 0 1 0 0 0 0 0 10 0;
    1 0 0 0 0 0 0 0 0 1;
    0 1 0 0 1 0 0 0 0 0;
    0 0 0 1 0 0 1 0 0 0;
    0 0 0 0 0 1 0 0 0 0;
    0 0 0 0 0 0 0 1 0 0];

m12_2 = M2(:,2) - M2(:,1); % reaction 1
m21_2 = -m12_2; % reaction 2
m34_2 = M2(:,4) - M2(:,3); % reaction 3
m43_2 = -m34_2; % reaction 4
m56_2 = M2(:,6) - M2(:,5); % etc.
m65_2 = -m56_2;
m78_2 = M2(:,8) - M2(:,7);
m87_2 = -m78_2;
m910_2 = M2(:,10) - M2(:,9);
m109_2 = -m910_2;

%Reaction vectors
R2 = [m12_2 m21_2 m34_2 m43_2 m56_2 m65_2 m78_2 m87_2 m910_2 m109_2]';
rank(R2)

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
y = [x(1)*x(3); x(4); x(1)*x(2); x(5); x(4); x(1)*x(6); x(5); x(7)*x(1); x(2)*x(1); x(3)*x(1)];

dxdt = -M*K*y;
end



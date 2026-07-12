close all
clear all
clc
%% 1. PLANT MODELING
% Component masses in kg
m_baterry = 0.125;  % Battery mass
m_arduino = 0.025;  % Arduino Uno mass
m_driver = 0.025;   % L298n driver mass
m_MPU = 0.002;      % MPU sensor mass
m_tacometer = 0.04; % Mass of each tachometer + its disc
m_wheel = 0.029;    % Mass of each wheel
m_motor = 0.034;    % Mass of each motor

g = 9.8;            % Gravitational acceleration [m/s^2]
b = 0.1;            % Linear friction
c = 0;              % Angular friction (neglected)
% --- PARAMETERS (1) ---
% Wheel data
mr = 2*m_wheel; 
r = 3.4/100;   
r_int = 2.5/100;
Ir = (1/2)*m_wheel*(r^2+r_int^2); % Moment of inertia

% Prism 1 data (Battery + Motors)
m1 = m_baterry + 2*m_motor + 2*m_tacometer;
h1 = 0.1/100; 

% Prism 2 data (Arduino + Drivers)
m2 = m_arduino + m_driver;
h2 = 5.2/100; 

% Prism 3 data (MPU Sensor)
m3 = m_MPU;
h3 = 10/100; 

% --- SIMPLIFIED PARAMETERS ---
mT = m1 + m2 + m3; 
Mh = m1*h1 + m2*h2 + m3*h3;
Ih = m1*h1^2 + m2*h2^2 + m3*h3^2;
M = mT + (3/2)*mr;
C1 = g*Mh;
delta = M*Ih-Mh^2;
%% 2. STATE-SPACE MODEL DEFINITION
a11 = 0;
a12 = 1;
a13 = 0;
a14 = 0;

a21 = 0;
a22 = -(Ih*b)/delta;
a23 = (Mh*C1)/delta;
a24 = -(Mh*c)/delta;

a31 = 0;
a32 = 0;
a33 = 0;
a34 = 1;

a41 = 0;
a42 = (Mh*b)/delta;
a43 = (M*C1)/delta;
a44 = -(M*c)/delta;

b1 = 0;
b2 = (Ih)/delta;
b3 = 0;
b4 = -(Mh)/delta;

A = [a11 a12 a13 a14; 
    a21 a22 a23 a24;
    a31 a32 a33 a34;
    a41 a42 a43 a44]; % System A matrix
B = [b1 b2 b3 b4]';   % System B matrix
C = [1 0 0 0];        % Taking x as input
D = 0;

% System order
[m,n] = size(A);

% % Controllability matrix
% M_C = ctrb(A,B);
% 
% % Rank of controllability matrix
% rango_MC = rank(M_C);
% 
% % Observability matrix
% N_C = obsv(A,C);
% 
% % Rank of observability matrix
% rango_NC = rank(N_C);
% 
% % Check if the system is completely observable
% if rango_MC ~= n % rank != n
%     fprintf('The continuous system is not completely controllable')
%     return
% end
% 
% % Check if the system is completely observable
% if rango_NC ~= n % rank != n
%     fprintf('The continuous system is not completely observable')
%     return
% end

%% 2. DISCRETIZATION
Ts = 0.01;  % Sampling time 10ms

sys_C = ss(A,B,C,D); % Continuous plant
sys_D = c2d(sys_C, Ts, 'zoh'); % Discrete plant

% Discrete State-Space matrices
G = sys_D.A; 
H = sys_D.B;
C = sys_D.C;
D = sys_D.D;

%% 3. CONTROLLABILITY AND OBSERVABILITY CHECK
if rank(ctrb(G, H)) < n
    error('The Discrete System is NOT controllable');
end
if rank(obsv(G, C)) < n
    error('The Discrete System is NOT observable');
end

%% 4. STATE FEEDBACK DESIGN

Q = diag([200, 10, 200, 20]); % High penalty for x and theta
R = 1; % Penalty for control effort

K = dlqr(G,H,Q,R);

%% 5. SIMULATION
% Parameters
T_total = 3;             
N = T_total / Ts;         

% Initialization
initial_angle = (pi/180)*10;
% Initial state: 10 degree tilt
X = [0 0 initial_angle 0]';   

% Simulation
for k = 1:N
    u = -K * X(:,k);  % Control law applied
    X(:,k+1) = G * X(:,k) + H * u;           
end

% Time vector
t = 0:Ts:T_total;

% Defined colors (you can add more if you have more states)
colors = [ ...
    0 0.4470 0.7410;        % blue
    0.8500 0.3250 0.0980;   % orange-red
    0.9290 0.6940 0.1250;   % yellow
    0.4940 0.1840 0.5560;   % purple
    0.4660 0.6740 0.1880;   % green
    0.3010 0.7450 0.9330;   % light blue
    0.6350 0.0780 0.1840];  % dark red

% Plot each state in a subplot
figure;
n = size(X, 1);  

for i = 1:n
    subplot(n, 1, i);               
    plot(t, X(i, :), 'LineWidth', 1.5, 'Color', colors(i, :));
    xlabel('Time [s]');
    ylabel(['x_' num2str(i)]);
    title(['State x_' num2str(i)]);
    grid on;
end

% General title
sgtitle('State Evolution with Controller');

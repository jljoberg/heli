% TTK4135 - Helicopter lab
% Hints/template for problem 2.
% Updated spring 2017, Andreas L. Fl?ten

%% Initialization and model definition
run init06.m; % NB: Change this to the init file corresponding to your helicopter

% Needs some global variables (pass to lonLinCon)
global N mx lambda_t alpha beta;

delta_t	= 0.25; % sampling time
% Discrete time system model. x = [lambda r p p_dot]'
%A1 = [0 1 0 0; [zeros(3,2), [-K_2 0; 0 1; -K_1*K_pp -K_1*K_pd]]];
A1_no_e = [1 delta_t 0 0; 0 1 -delta_t*K_2 0; 0 0 1 delta_t; 0 0 -delta_t*K_1*K_pp 1-delta_t*K_1*K_pd];
A1 = [A1_no_e, zeros(4,2); zeros(2,4), [1, delta_t ; -delta_t*K_3*K_ep, 1-delta_t*K_3*K_ed]];
B1 = [0 0 0 delta_t*K_1*K_pp 0 0 ; 0 0 0 0 0 K_3*K_ep]';
%B1 = [0 0 0 K_1*K_pp]';

% Number of states and inputs
mx = size(A1,2); % Number of states (number of columns in A)
mu = size(B1,2); % Number of inputs(number of columns in B)

% Initial values
x1_0 = pi;                              % Lambda
x2_0 = 0;                               % r
x3_0 = 0;                               % p
x4_0 = 0;                               % p_dot
x5_0 = 0;                               % e
x6_0 = 0;                               % e_dot
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0 x6_0]';  % Initial values
lambda_startOffset = 180;

% Time horizon and initialization
N  = 40;                                % Time horizon for states
M  = N;                                 % Time horizon for inputs
z  = zeros(N*mx+M*mu,1);                % Initialize z for the whole horizon
z0 = z;                                 % Initial value for optimization

% Bounds
ul 	    = -30*pi/180;                   % Lower bound on control -- u1
uu 	    = 30*pi/180;                    % Upper bound on control -- u1

xl      = -Inf*ones(mx,1);              % Lower bound on states (no bound)
xu      = Inf*ones(mx,1);               % Upper bound on states (no bound)
xl(3)   = ul;                           % Lower bound on state x3
xu(3)   = uu;                           % Upper bound on state x3

% Generate constraints on measurements and inputs
[vlb,vub]       = genBegr2(N,M,xl,xu,ul,uu); % hint: genbegr2
vlb(N*mx+M*mu)  = 0;                    % We want the last input to be zero
vub(N*mx+M*mu)  = 0;                    % We want the last input to be zero

% Generate the matrix Q and the vector c (objecitve function weights in the QP problem) 
%Q1 = zeros(mx,mx); 
Q1 = diag([1 1 1 1 1 1]);
%Q1(1,1) = ;                             % Weight on state x1
%Q1(2,2) = ;                            % Weight on state x2
%Q1(3,3) = ;                             % Weight on state x3
%Q1(4,4) = ;                            % Weight on state x4
P1 = diag([0 0]);                                 % Weight on input
Q = 2*genq2(Q1,P1,N,M,mu);              % Generate Q
c = zeros(N*mx+M*mu,1);                 % Generate c

%% Generate system matrixes for linear model
Aeq = gena2(A1,B1,N,mx,mu);           % Generate A, hint: gena2
beq = zeros(N*mx,1);        	  % Generate b
beq(1:mx) = A1*x0; % Initial value

%% Solve QP problem with linear model
tic
H_diag = [repmat([1 0 0 0 0 0]', N,1); repmat([0.1 0.1]', M,1)];
H = diag(H_diag);


objectFun = @(z) z'*H*z;
alpha = 0.2;
beta = 20;
lambda_t = (2*pi)/3;

options = optimoptions('fmincon', 'MaxFunEvals', 1000000);

x0_fmin = zeros(N*mx+M*mu ,1);
x0_fmin(1:mx) = x0;
% [z,lambda] = quadprog(H, [], [], [], Aeq, beq, vlb, vub); % hint: quadprog
[z, fval] = fmincon(objectFun, x0_fmin, [], [], Aeq, beq, vlb, vub, @nonLinCon, options);
t1=toc;

% Calculate objective value
phi1 = 0.0;
PhiOut = zeros(N*mx+M*mu,1);
for i=1:N*mx+M*mu
  phi1=phi1+Q(i,i)*z(i)*z(i);
  PhiOut(i) = phi1;
end

%% Extract control inputs and states
%u  = [z(N*mx+1:N*mx+M*mu)]; % Control input from solution
u1 = [z((N*mx+1):mu:N*mx+M*mu)];              % State x1 from solution
u2 = [z((N*mx+2):mu:N*mx+M*mu)];              % State x1 from solution

x1 = [x0(1);z(1:mx:N*mx)];              % State x1 from solution
x2 = [x0(2);z(2:mx:N*mx)];              % State x2 from solution
x3 = [x0(3);z(3:mx:N*mx)];              % State x3 from solution
x4 = [x0(4);z(4:mx:N*mx)];              % State x4 from solution
x5 = [x0(4);z(5:mx:N*mx)];              % State x5 from solution
x6 = [x0(4);z(6:mx:N*mx)];              % State x6 from solution

num_variables = 5/delta_t;
zero_padding = zeros(2*num_variables,1);
unit_padding  = ones(2*num_variables,1);

u1   = [zero_padding; u1; zero_padding];
u2   = [zero_padding; u2; zero_padding];
x1  = [pi*unit_padding; x1; zero_padding];
x2  = [zero_padding; x2; zero_padding];
x3  = [zero_padding; x3; zero_padding];
x4  = [zero_padding; x4; zero_padding];
x5  = [zero_padding; x5; zero_padding];
x6  = [zero_padding; x6; zero_padding];

%% Plotting
%{
t = 0:delta_t:delta_t*(length(u)-1);

figure(2)
subplot(511)
stairs(t,u),grid
ylabel('u')
subplot(512)
plot(t,x1,'m',t,x1,'mo'),grid
ylabel('lambda')
subplot(513)
plot(t,x2,'m',t,x2','mo'),grid
ylabel('r')
subplot(514)
plot(t,x3,'m',t,x3,'mo'),grid
ylabel('p')
subplot(515)
plot(t,x4,'m',t,x4','mo'),grid
xlabel('tid (s)'),ylabel('pdot')

%print -depsc Task_2.3_q_00.1
%}


%% ======================================================================================
%%=================================== TASK 3 BEGIN ======================================
%%=======================================================================================
%% QR controller

lambdaCost = 350;
rCost = 1;
pCost = 1;
p_dotCost = 1;
input_pitchCost = 50;

Q_lq = diag([lambdaCost, rCost, pCost, p_dotCost 0 0]);
R_lq = diag([input_pitchCost 0.001]);

K_lq_temp = dlqr(A1, B1, Q_lq, R_lq);
K_lq = K_lq_temp(1,1:4);

%% Create accepable input to simulink
uTime = linspace(0, size(u1,1)*0.25 - 0.25, size(u1,1));
u1Input = [uTime' u1];
u2Input = [uTime' u2];

xTime = linspace(0, size(x1,1)*0.25 - 0.25, size(x1,1));
xInput = [xTime', x1, x2, x3, x4];




clear all;
close all;
clc;
fclose all;
bdclose all;
desired_state=0;

% cost function
Q = [100,0;0,5];
R = 0.5;

%control interval
del_t = 0.5;
w = 10;

% dynamics equation x(n+1) = A*x(n) + B*u
A = [1,del_t;w*del_t,1];
B = [0;-w*del_t];

%feedforward and feedback const
K= struct;
k = struct;

P_next = Q;
p_next = -Q*[10;0];
horizon_len = floor(12/del_t);
der_state_array =zeros(horizon_len,2);

%desired state at timestep
for i = 1:horizon_len
del_t = 0.5;
%global desired_state;
r = randi([1 9],1);
v = 0.4;%r/10;%0.7;%r/10;%0.4;%r/10;
desired_state = desired_state+v*del_t;
der_state_array(i,:) = [desired_state;0]; 

end

%get feedforward and feeback constant using backward Riccatti Equation
for n = horizon_len:-1:1

qn = -Q*der_state_array(n,:)';%state_der(desired_state,n);
Kn = -inv(R + B'*P_next*B)*B'*P_next*A;
Pn = Q + A'*P_next*A + A'*P_next*B*Kn;
kn = -inv(R + B'*P_next*B)*B'*p_next;
pn = qn + A'*p_next + A'*P_next*B*kn;
K(n).value = Kn;
k(n).value = kn;
P_next = Pn;
p_next = pn;
end

% ra = randi([0 1],1);
ra = 0; %without Disturbance
% ra = 1;


%take the controls and simulate a full inverted pendulum
WalkingManSimulation(K,k,del_t,ra)

%% plots


% desired COM vs Actual COM

tRun = 0:del_t:(size(der_state_array,1)*del_t)-del_t;
tRun = tRun';
tRun1 = 0:0.001:10;
tRun1(end) = [];
tRun1 = tRun1';
x_com_curr = evalin('base','x_com_curr_StrongStore');

figure('Name', 'Center of Mass Position')
plot(tRun, der_state_array(:,1)); 
hold on
plot(tRun1,x_com_curr);
legend('x-desired', 'x-actual')
hold off
ylabel('COM Position')
xlabel("Time")

strideLen = evalin('base','strideLen_StrongStore');
acc = evalin('base','acc_StrongStore');

% stride len and acc with time
figure('Name', 'Stride length v/s Acceleration')
plot(tRun1, strideLen, tRun1,  acc)
legend('Stride Length', 'Acceleration')

%vel 

velocityNew = evalin('base','x_com_vel_StrongStore');
figure('Name', 'COM Velocity')
plot(tRun1, velocityNew)
ylabel('COM Velocity')
xlabel("Time")























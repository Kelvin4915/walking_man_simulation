function WalkingManSimulation(K,k,control_Interval,distCond)
theta=0;
omega = 0;
horizon_len = 10;
%frequency of animation
del_t = 0.001;
w = 10;
thetaSolPlot = zeros(horizon_len*100,1);
axis(gca,'equal');
axis([-2 10 -2 2]);
grid on;
xcom_prev=0;
x_com_vel=0;
ycom_prev =1;
y_com_vel=0;
cop_idx=1;
xcop_prev=0;
xcop=0;
left=1;
x_com_curr_StrongStore = [];
strideLen_StrongStore = [];
acc_StrongStore = [];
x_com_vel_StrongStore = [];
for h = 1:horizon_len*1000
% apply controls to the simulations
    if(mod(h,floor(control_Interval/del_t))==1)
        xcop_prev = xcop;
        xcop = K(cop_idx).value*[xcom_prev;x_com_vel]+k(cop_idx).value;
        cop_idx = cop_idx+1;
        left = mod(left+1,2);
    end

    strideLen_StrongStore(end+1,:) = abs(xcop_prev - xcop);
    assignin('base',"strideLen_StrongStore",strideLen_StrongStore);
    theta = asin((-xcop+xcom_prev));
    thetaSolPlot(h) = theta;

    %apply distrubance if condition is true
    if ~distCond
        ang_acc = w*sin(theta);
    else
        ang_acc = w*sin(theta);
        r = randi([-1 1],1);
        r = r/10;
        ang_acc = ang_acc + r;
    end
    x_com_acc = ang_acc*cos(theta);
    acc_StrongStore(end+1,:) = x_com_acc;
    assignin('base',"acc_StrongStore",acc_StrongStore);

    %full pendulum dynamics
    x_com_vel = x_com_vel + x_com_acc*del_t;
    x_com_curr = xcom_prev+x_com_vel*del_t+0.5*x_com_acc*(del_t^2);
    x_com_curr_StrongStore(end+1,:) = x_com_curr; %#ok<AGROW>
    assignin('base',"x_com_curr_StrongStore",x_com_curr_StrongStore);
    step_size=x_com_curr-xcop;
    y_com_curr = abs(sqrt(1-step_size*step_size));
    x_com_vel_StrongStore(end+1,:) = x_com_vel;
    assignin('base',"x_com_vel_StrongStore",x_com_vel_StrongStore);

    %plot the legs for simulation
    if(left)
        leg1 = line([xcop x_com_curr],[0 y_com_curr],'Color','red');
        leg2 = line([xcop_prev x_com_curr],[0 y_com_curr]);
    else
        leg1 = line([xcop x_com_curr],[0 y_com_curr]);
        leg2 = line([xcop_prev x_com_curr],[0 y_com_curr],'Color','red');
    end
    xcom_prev=x_com_curr;
    ycom_prev = y_com_curr;

    %frequency of animation
    pause(del_t);
    delete(leg1);
    delete(leg2);

end
close(gcf)

end

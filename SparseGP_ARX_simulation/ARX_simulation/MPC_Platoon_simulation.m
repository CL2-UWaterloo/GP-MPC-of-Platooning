clear all; close all; clc; 

% MPC variables 
N = 6;    % MPC Horizon
dt = 1/4; % sample time
M = 2; % No of autonomous vehicles
 
% constraints
% constraints, https://copradar.com/chapts/references/acceleration.html
a_max = 5;  % max accl, Top of the line production muscle cars is 0.55 g.
a_min = -5; % min accl, 0.7 g is the Vehicle Max, 0.47 g is the Average Driver Max
v_max = 35;  % max vel, 126km/h
v_min = -35; % min vel
delta = 20;  % safe distance between vehicles

% initializations
v0 = zeros(1,M); % velocity varibales of AVs
p0 = zeros(1,M); % position variables of AVs
% a0 = zeros(1,M); % acceleration variables of AVs

% initialize AV positions
for m = 2:M
    p0(m) = p0(m-1)-1.2*delta; % start 2 delta behind car in front
end
p0_h  = p0(M)- 1.2*delta; % HV starts 2 delta behind the last AV
v0_h = 0; % velocity initials of HV
v0_h_gp = v0_h;
p0_h_gp = p0_h;

% past 4 samples (set to zero for simplicity) of velocity for the last AV
v_M_m1 = 0; % i-1, i-2, i-3 and i-4
v_M_m2 = 0;
v_M_m3 = 0;
v_M_m4 = 0;
% past 4 samples (set to zero for simplicity) of velocity for the HV
v_h_m1 = 0;
v_h_m2 = 0;
v_h_m3 = 0;
v_h_m4 = 0;

Q = 5; % MPC weight for velocity tracking 
R = 20; % weight for control 
% prob_desired = 0.95; % desired probability for tighenting distance constraint
% prob_desired = 0.997; % 3\sigma

%% Simulation 
sim_tim = 60; % simulation time

t0 = 0; % initial time step
TIME = sim_tim / dt; % simualtion steps
ET = zeros(TIME, 1); % mpc runing time
t = zeros(M, 1);
t(1) = t0;

k = 0;  % system simulation time step k
av_accel = zeros(TIME,M); % accelerations hisotry from MPC
vv0 = [v0;zeros(TIME,M)]; % velocity history of AVs
vv0_h = [v0_h;zeros(TIME-1,1)]; % velocity history of HV for ARX norminal model updates
vv0_h_gp = [v0_h;zeros(TIME-1,1)]; % velocity history of HV with GP corrections

pp0 = [p0;zeros(TIME,M)]; % position history of AVs
pp0_h_gp = [p0_h;zeros(TIME,1)]; % position history of HV with ARX + GP

AV_accel = zeros(N,M); % MPC acceleration output
AV_vel = zeros(N+1,M);

mpc_cost = zeros(TIME,1); % all mpc cost J

humanvar_offset = (N)*M+(N+1)*2*M; % the first these many variables are for the AVs: N*M for accel and (N+1)*M for vel and pos each

while k < TIME
    % init variables for past and limit them to be equal to their measured vals
    lbvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
    ubvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
    lbvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
    ubvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
    
    % reference velocity 
    if k< TIME/2
        v_ref = 20*ones(N+1,1); 
%     elseif k< 2*TIME/3
%         v_ref = 15*ones(N+1,1);
%     elseif k < 5*TIME/6
%         v_ref = 10*ones(N+1,1);
    else
        v_ref = 10*ones(N+1,1);
    end

    % MPC
    tic         

    [sol] = MPC_platoon(dt, N, M, v_ref, delta, Q, R, ...
                        v_min, v_max, a_max, a_min, ...
                        p0, v0, v0_h_gp, p0_h_gp, ...
                        lbvM_, ubvM_, lbvh_, ubvh_);

    % update the history for next time steps
    % update k-2, k-3, k-4 
    % Note: every time to call the MPC loop, we need to update the initials,
    % including the k-1, k-2, k-3, k-4 at the begining
    v_M_m4 = v_M_m3; 
    v_M_m3 = v_M_m2;
    v_M_m2 = v_M_m1;
    v_h_m4 = v_h_m3; 
    v_h_m3 = v_h_m2;
    v_h_m2 = v_h_m1;

    % for k-1 assign the current vel to v_M_m1 for the next iteration
    v_M_m1 = v0(end); % for last AV in platoon
    v_h_m1 = v0_h_gp; % v0_h_gp is the actual "measured" states from the plant
                      % we use the arx+gp model to represent the real plant
                      % behaviors. The model propagations must be handled
                      % by the arx model, gp is the corrections to make
                      % the whole system model more accurate to the real
                      % plant.  
    
    % retrive results from the solution
    for m = 1:M
       AV_accel(:,m) = full(sol.x((N+1)*2*M+1+(N)*(m-1):(N)*m+(N+1)*2*M));

       AV_vel(:,m) = full(sol.x((N+1)*M+1+(N+1)*(m-1):(N+1)*m+(N+1)*M));
    end

    % MPC cost J
    J = 0; % cost function
    % lead vehicle tracking cost
    J = J + Q*(v_ref(1)-AV_vel(1,1))^2;
    % follower tracking costs
    for m = 2:M
        J = J + Q*(AV_vel(1,m)-AV_vel(1,m-1))^2;
    end
    % acceleration cost
    % lead vehicle tracking cost
    J = J + R*(AV_accel(1,1))^2;
    % follower tracking costs
    for m = 2:M
        J = J + R*(AV_accel(1,m))^2;
    end
    
    mpc_cost(k+1, :) = J;

    t(k+1) = t0;
    
    toc
    ET(k+1) = toc;

    % calculate the new initial values for MPC 
    [t0, p0, v0, v0_h, v0_h_gp, p0_h_gp] = GP_sysmodel(dt, k+1, t0, p0, vv0, vv0_h, vv0_h_gp, pp0_h_gp, M, AV_accel);

    % convert casadi.DM to double 
    v0_h = full(v0_h);
    v0_h_gp = full(v0_h_gp);
    p0_h_gp = full(p0_h_gp);

    % save variable history 
    vv0(k+2, :) = v0;
    vv0_h(k+1,:) = v0_h;
    vv0_h_gp(k+1, :) = v0_h_gp;
    pp0(k+2, :) = p0;
    pp0_h_gp(k+2, :) = p0_h_gp;
    av_accel(k+1, :)= AV_accel(1,:); % the first elements of accel 

    k = k + 1;
end

%% Reconstruct stuff from solver
AV_positions = pp0;
AV_velocities = vv0;
AV_accelerations = av_accel;
H_positions_gp = pp0_h_gp;

H_velocities = vv0_h;
H_velocities_gp = vv0_h_gp;
t = [t; t(end)+dt];
%% Plot 
v_reference = 20*ones(length(t),1); % vref for lead vehicle

subplot(411) %plot velocities
plot(t,v_reference,'k--');hold on;grid on;
plot(t,AV_velocities(:,1),'b'); hold on;grid on;
for m = 2:M
  plot(t,AV_velocities(:,m),'r-.'); 
end
plot(t(1:length(t)-1),H_velocities,'y');hold on;grid on;
plot(t(1:length(t)-1),H_velocities_gp,'g');hold on;grid on;
legend(["velocity ref", "leading AV velocity", "AVs velocity", "arx HV vel", "arx+GP HV vel"],'Location','best');

xlabel('Time steps');ylabel('Velocities');
xlim([0 t(end)])

subplot(412)
plot(t,AV_positions(:,1),'b'); hold on;grid on;
for m = 2:M
  plot(t,AV_positions(:,m),'r-.'); 
end
plot(t,H_positions_gp,'g');hold on;grid on;
legend(["leading AV position", "AVs position", "arx HV position"],'Location','best');

xlabel('Time steps');ylabel('Positions');
xlim([0 t(end)])

subplot(413)
plot(t,AV_positions(:,1) - AV_positions(:,2),'b');hold on;grid on;
if(M>2) 
   for m = 2:M-1
      plot(t,AV_positions(:,m) - AV_positions(:,m+1),'r-.'); 
   end
end

plot(t,AV_positions(:,M) - H_positions_gp,'g');hold on;grid on;
legend(["lead-follower AV relative pos", "AV-HV relative pos"],'Location','best');

xlabel('Time steps');ylabel('Relative distance');
xlim([0 t(end)])

subplot(414)
plot(t(1:length(t)-1),AV_accelerations(:,1),'b');hold on;grid on;

for m = 2:M
  plot(t(1:length(t)-1),AV_accelerations(:,m),'r-.'); 
end
legend(["leading AV accel", "AV accel"],'Location','best');

xlabel('Time steps');ylabel('Accl');
xlim([0 t(end)])

%% Debug MPC loop
% 
% v_ref(1:floor((N+1)/2)) = 20; 
% v_ref(ceil((N+1)/2):N+1) = 20;
% % init variables for past and limit them to be equal to their measured vals
% lbvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
% ubvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
% lbvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
% ubvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
% 
% [sol] = MPC_platoon(dt, N, M, v_ref, delta, Q, R, ...
%                     v_min, v_max, a_max, a_min, ...
%                     p0, v0, v0_h_gp, p0_h_gp, ...
%                     lbvM_, ubvM_, lbvh_, ubvh_);
% 
% 
% %Reconstruct stuff from solver
% AV_positions = zeros(N+1,M);
% AV_velocities = zeros(N+1,M);
% AV_accelerations = zeros(N,M);
% H_positions = zeros(N+1,1);
% H_velocities = zeros(N+1,1);
% 
% H_velocities_gp = zeros(N+1,1);
% 
% for m = 1:M
%    AV_positions(:,m) = full(sol.x(1+(N+1)*(m-1):(N+1)*m));
%    AV_velocities(:,m) = full(sol.x((N+1)*M+1+(N+1)*(m-1):(N+1)*m+(N+1)*M));
%    AV_accelerations(:,m) = full(sol.x((N+1)*2*M+1+(N)*(m-1):(N)*m+(N+1)*2*M));
% end
% 
% humanvar_offset = (N)*M+(N+1)*2*M; % the first these many variables are for the AVs: N*M for accel and (N+1)*M for vel and pos each
% H_positions = full(sol.x(humanvar_offset+1:humanvar_offset+N+1)); 
% H_velocities = full(sol.x(humanvar_offset+N+1+1:humanvar_offset+N+1+(N+1))); 
% %%
% % Plot stuff only for 2 AVs
% subplot(311) %plot velocities
% plot(1:N+1,v_ref,'k--');hold on;grid on;
% plot(1:N+1,AV_velocities(:,1),'b');
% if(m>1)
%    for m = 2:M
%       plot(1:N+1,AV_velocities(:,m),'r-.'); 
%    end
% end
% plot(1:N+1,H_velocities,'r'); hold on;grid on;
% plot(1:N+1,H_velocities_gp,'g');
% xlabel('Time steps');ylabel('Velocities');
% xlim([0 N+1])
% 
% subplot(312)
% plot(1:N+1,H_positions,'g');hold on;grid on;
% plot(1:N+1,AV_positions(:,1),'b');
% if(m>1)
%    for m = 2:M
%       plot(1:N+1,AV_positions(:,m),'r-.'); 
%    end
% end
% xlabel('Time steps');ylabel('Positions');
% xlim([0 N+1])
% 
% subplot(313)
% plot(1:N,AV_accelerations(:,1),'b');hold on;grid on;
% if(m>1)
%    for m = 2:M
%       plot(1:N,AV_accelerations(:,m),'r-.'); 
%    end
% end
% xlabel('Time steps');ylabel('Accl');
% xlim([0 N+1])

%%
%%
%% Simulation 
% sim_tim = 30; % simulation time
% 
% t0 = 0;
% TIME = sim_tim / dt; % simualtion steps
% t = zeros(M, 1);
% t(1) = t0;
% 
% % Start MPC
% k = 0;  % system simulation time step k
% av_accel = zeros(TIME,M);
% vv0 = [v0;zeros(TIME,M)];
% vv0_h = [v0_h;zeros(TIME-1,1)];
% pp0 = [p0;zeros(TIME,M)];
% pp0_h = [p0_h;zeros(TIME,1)];
% 
% AV_accel = zeros(N,M);
% 
% humanvar_offset = (N)*M+(N+1)*2*M; % the first these many variables are for the AVs: N*M for accel and (N+1)*M for vel and pos each
% 
% while k < TIME
% 
%     % init variables for past and limit them to be equal to their measured vals
%     lbvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
%     ubvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
%     lbvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
%     ubvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
% 
%     if k< 1*TIME/2
%         v_ref = 15*ones(N+1,1); 
% %     elseif k< 2*TIME/6
% %         v_ref = 20*ones(N+1,1);
% %     elseif k < 4*TIME/6
% %         v_ref = 40*ones(N+1,1);
%     else
%         v_ref = 10*ones(N+1,1);
%     end
% 
%     [sol] = MPC_platoon(dt, N, M, v_ref, delta, Q, R, ...
%                         v_min, v_max, a_max, a_min, ...
%                         p0, v0, v0_h, p0_h, ...
%                         lbvM_, ubvM_, lbvh_, ubvh_);
% 
%     % update the history for next time steps
%     % update k-2, k-3, k-4 
%     % Note: every time to call the MPC loop, we need to update the initials,
%     % including the k-1, k-2, k-3, k-4 at the begining
%     v_M_m4 = v_M_m3; 
%     v_M_m3 = v_M_m2;
%     v_M_m2 = v_M_m1;
%     v_h_m4 = v_h_m3; 
%     v_h_m3 = v_h_m2;
%     v_h_m2 = v_h_m1;
%     
%     % for k-1 assign the current vel to v_M_m1 for the next iteration
%     v_M_m1 = v0(end); %for last AV in platoon
%     v_h_m1 = v0_h;
%     
%     % retrive results from the solution
%     for m = 1:M
%        AV_accel(:,m) = full(sol.x((N+1)*2*M+1+(N)*(m-1):(N)*m+(N+1)*2*M));
%     end
% 
%     t(k+1) = t0;
%     % calculate the new initial values for MPC 
%     [t0, p0, v0, v0_h, p0_h] = sysmodel(dt, k+1, t0, p0, vv0, vv0_h, p0_h, M, AV_accel);
% 
%     vv0(k+2, :) = v0;
%     vv0_h(k+1,:) = v0_h;
%     pp0(k+2, :) = p0;
%     pp0_h(k+2, :) = p0_h;
%     av_accel(k+1, :)= AV_accel(1,:); % the first elements of accel 
% 
%     k = k + 1;
% end
% 
% % Reconstruct stuff from solver
% AV_positions = pp0;
% AV_velocities = vv0;
% AV_accelerations = av_accel;
% H_positions = pp0_h;
% H_velocities = vv0_h;
% 
% t = [t; t(end)+dt];
% % Plot 
% % v_reference = 20*ones(length(t),1); % vref for lead vehicle
% 
% subplot(411) %plot velocities
% % plot(t,v_ref,'k--');hold on;grid on;
% plot(t,AV_velocities(:,1),'b'); hold on;grid on;
% for m = 2:M
%   plot(t,AV_velocities(:,m),'r-.'); 
% end
% plot(t(1:length(t)-1),H_velocities,'g');
% xlabel('Time steps');ylabel('Velocities');
% xlim([0 t(end)])
% 
% subplot(412)
% plot(t,AV_positions(:,1),'b'); hold on;grid on;
% for m = 2:M
%   plot(t,AV_positions(:,m),'r-.'); 
% end
% plot(t,H_positions,'g');hold on;grid on;
% xlabel('Time steps');ylabel('Positions');
% xlim([0 t(end)])
% 
% subplot(413)
% plot(t,AV_positions(:,1) - AV_positions(:,2),'b');hold on;grid on;
% if(M>2) 
%    for m = 2:M-1
%       plot(t,AV_positions(:,m) - AV_positions(:,m+1),'r-.'); 
%    end
% end
% plot(t,AV_positions(:,M) - H_positions,'g');hold on;grid on;
% xlabel('Time steps');ylabel('Relative distance');
% xlim([0 t(end)])
% 
% subplot(414)
% plot(t(1:length(t)-1),AV_accelerations(:,1),'b');hold on;grid on;
% 
% for m = 2:M
%   plot(t(1:length(t)-1),AV_accelerations(:,m),'r-.'); 
% end
% 
% xlabel('Time steps');ylabel('Accl');
% xlim([0 t(end)])
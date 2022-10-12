clear all; close all; clc; 

% MPC variables 
N = 10;  % MPC Horizon
dt = 0.1;   % sample time
M = 2; % No of autonomous vehicles
 
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
    p0(m) = p0(m-1)-1.0*delta; % start 2 delta behind car in front
end
p0_h  = p0(M)- 1.0*delta; % HV starts 2 delta behind the last AV
v0_h = 0; % velocity initials of HV
v0_h_gp = v0_h;
p0_h_gp = p0_h;
p0_h_sigma = 0; % position variance initials of HV 

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
R = 10; % weight for control 
prob_desired = 0.95; % 2\sigma

%% Simulation 
sim_tim = 1; % simulation time

t0 = 0; % initial time step
TIME = sim_tim / dt; % simualtion steps
t = zeros(M, 1);
t(1) = t0;

k = 0;  % system simulation time step k
av_accel = zeros(TIME,M); % accelerations hisotry from MPC
h_pos_sigma = zeros(TIME, 1);

vv0 = [v0;zeros(TIME,M)]; % velocity history of AVs
vv0_h = [v0_h;zeros(TIME-1,1)]; % velocity history of HV for ARX norminal model updates
vv0_h_gp = [v0_h;zeros(TIME-1,1)]; % velocity history of HV with GP corrections

pp0 = [p0;zeros(TIME,M)]; % position history of AVs
pp0_h = [p0_h;zeros(TIME,1)]; % position history of HV with ARX norminal model 
pp0_h_gp = [p0_h;zeros(TIME,1)]; % position history of HV with ARX + GP
pp0_h_sigma = [p0_h_sigma;zeros(TIME,1)]; % position variance history of HV 

MPC_cost = zeros(TIME,1); % all mpc cost JJ

AV_accel = zeros(N,M); % MPC acceleration output
AV_vel = zeros(N+1,M);
% H_velocities = zeros(N+1,1);
H_positions_sigma = zeros(N+1, 1);
humanvar_offset = (N)*M+(N+1)*2*M; % the first these many variables are for the AVs: N*M for accel and (N+1)*M for vel and pos each

while k < TIME
    % init variables for past and limit them to be equal to their measured vals
    lbvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
    ubvM_ = [v_M_m1;v_M_m2;v_M_m3;v_M_m4];
    lbvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
    ubvh_ = [v_h_m1;v_h_m2;v_h_m3;v_h_m4];
    
    % reference velocity 
    if k< TIME/2
        v_ref = 20*ones(N+1,1);   % 90 km/h
%     elseif k< TIME/2
%         v_ref = 40*ones(N+1,1);
%     elseif k < 5*TIME/6
%         v_ref = 10*ones(N+1,1);
    else
        v_ref = 10*ones(N+1,1);
    end

    % MPC
    [sol] = GP_MPC_platoon(dt, N, M, v_ref, delta, Q, R, ...
                           v_min, v_max, a_max, a_min, ...
                           p0, v0, v0_h_gp, p0_h_gp, p0_h_sigma, prob_desired,...
                           lbvM_, ubvM_, lbvh_, ubvh_);
    
%     sigma = full(sigma);

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
    
    % retrive results from the solution
    for m = 1:M
%        AV_positions(:,m) = full(sol.x(1+(N+1)*(m-1):(N+1)*m));
       AV_vel(:,m) = full(sol.x((N+1)*M+1+(N+1)*(m-1):(N+1)*m+(N+1)*M));
       AV_accel(:,m) = full(sol.x((N+1)*2*M+1+(N)*(m-1):(N)*m+(N+1)*2*M));
    end

    t(k+1) = t0;

    % calculate the new initial values for MPC 
    [t0, p0, v0, v0_h, v0_h_gp, p0_h_gp] = GP_sysmodel(dt, k+1, t0, p0, vv0, vv0_h, vv0_h_gp, pp0_h_gp, M, AV_accel);
    % convert casadi.DM to double 
    v0_h = full(v0_h);
    v0_h_gp = full(v0_h_gp);
    p0_h_gp = full(p0_h_gp);

    % MPC cost JJ
    JJ = 0; % cost function
    % lead vehicle tracking cost
    JJ = JJ + Q*(v_ref(1)-v0(1,1))^2;
    % follower tracking costs
    for m = 2:M
        JJ = JJ + Q*(v0(1,m)-v0(1,m-1))^2;
    end
    % acceleration cost
    % lead vehicle tracking cost
    JJ = JJ + R*(AV_accel(1,1))^2;
    % follower tracking costs
    for m = 2:M
        JJ = JJ + R*(AV_accel(1,m))^2;
    end
    
    MPC_cost(k+1, :) = JJ;

    % save variable history 
    vv0(k+2, :) = v0;
    vv0_h(k+1,:) = v0_h;
    vv0_h_gp(k+1, :) = v0_h_gp;
    pp0(k+2, :) = p0;
    pp0_h_gp(k+2, :) = p0_h_gp;
    av_accel(k+1, :)= AV_accel(1,:); % the first elements of accel 

%     h_pos_sigma(k+1, :) = H_positions_sigma(1,1);

    k = k + 1;
end

%% Reconstruct stuff from solver
AV_positions = pp0;
AV_velocities = vv0;
AV_accelerations = av_accel;
H_positions_gp = pp0_h_gp;
H_positions_gp_var = pp0_h_sigma;

H_velocities = vv0_h;
H_velocities_gp = vv0_h_gp;
t = [t; t(end)+dt];
%% Plot 
v_reference(1:floor(length(t)/2)) = 20; 
v_reference(ceil(length(t)/2):length(t)) = 10;
subplot(411) %plot velocities
plot(t,v_reference,'k--');hold on;grid on;
plot(t,AV_velocities(:,1),'b', 'LineWidth',1.5); hold on;grid on;
for m = 2:M
  plot(t,AV_velocities(:,m),'r-.', 'LineWidth',1.5); 
end
% plot(t(1:length(t)-1),H_velocities,'c--');hold on;grid on;
plot(t(1:length(t)-1),H_velocities_gp,'g','LineWidth',1.5);hold on;grid on;

legend(["velocity ref", "leading AV velocity", "AVs velocity","arx+GP HV vel"],'Location','best');
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

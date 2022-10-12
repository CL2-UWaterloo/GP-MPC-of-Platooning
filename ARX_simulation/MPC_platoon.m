function [sol] = MPC_platoon(dt, N, M, v_ref, delta, Q, R, ...
                            v_min, v_max, a_max, a_min, ...
                            p0, v0, v0_h, p0_h,...
                            lbvM_, ubvM_, lbvh_, ubvh_)

    addpath('/home/jwang/Docs/Casadi/casadi-linux-matlabR2014b-v3.5.5')
    import casadi.*

    % ARX difference model parameters
    a1 = -3.0227; a2 = 3.3543; a3 = -1.6329; a4 = 0.3014; 
    b1 = 0.0063; b2 = -0.0303; b3 = 0.0495; b4 = -0.0254;
        
    % variables
    v = MX.sym('v',N+1,M); % vel of AVs
    p = MX.sym('p',N+1,M); % pos of AVs
    a = MX.sym('a',N,M); % accl of AVs 
    
    % HV ARX norminal and ARX norminal + GP 
    vh = MX.sym('vh',N+1,1); 
    ph = MX.sym('ph',N+1,1); 
  
    % variables for history
    vM_ = MX.sym('vM_',4,1); 
    vh_ = MX.sym('vh_',4,1);

    % Casadi optimize the general problem
    %      min_x   f(x)
    %     x \in R
    %  s.t. lbx<=x<=ubx
    %       lbg<=g(x)<=lbg
    
    % start with empty
    lbg = [];
    ubg = [];
    g = [];
    J = 0; % cost function to minimize
    for i = 1:N+1 % loop over time steps in horizon
        % lead vehicle tracking cost
        J = J + Q*(v_ref(i)-v(i,1))^2;
        if i < N+1  % punish v change
            J = J + Q*(v(i+1,1)-v(i,1))^2;
        end
        % follower tracking costs
        for m = 2:M
            J = J + Q*(v(i,m)-v(i,m-1))^2;
            if i < N+1  % punish v change
                J = J + Q*(v(i+1,m)-v(i,m))^2;
            end
        end
        
        if i < N+1
            % acceleration cost
            % lead vehicle tracking cost
            J = J + R*(a(i,1))^2;
            if i < N  % punish a change
                J = J + R*(a(i+1,1)-a(i,1))^2;
            end
            % follower tracking costs
            for m = 2:M
                J = J + R*(a(i,m))^2;
                if i < N  % punish a change
                    J = J + R*(a(i+1,m)-a(i,m))^2;
                end
            end
        else
            J = J;
        end
        
        % HVs systme model 
        if(i>1) % after the first time step
            for m = 1:M
                % velocity dynamics (equality constraint)
                g = [g; v(i,m) - v(i-1,m) - dt*a(i-1,m)];
                lbg = [lbg;0]; % min and max are both 0, defining a equality constraint
                ubg = [ubg;0];
                % position dynamics (equality constraint)
                g = [g; p(i,m) - p(i-1,m) - dt*v(i-1,m)];
                lbg = [lbg;0];
                ubg = [ubg;0];
                
                if(m>1) % if not lead AV
                    % m^th AV is at least delta m ahead of m-1^th AV
                    g = [g; p(i,m-1)-p(i,m)];
                    lbg = [lbg; delta]; 
                    ubg = [ubg; 10^6]; % large +'ve number instead of inf
                end
            end
        else % initial time step
            %v(i,:) = v0; % enforced via lbv and lbp
            %p(i,:) = p0;
        end
        
        % human vehicle dynamics for the first 4 time steps in horizon
        if(i==1) % at first time step in horizon. This is initialized later, have to be commented to avoid dimension issue. 
            %vh(i) = v0_h; 
        end 
        % refactor these into  _g(constants)=<g(x)<=g_(constants), casadi prefer a constant min and max rather than equations
        if(i==2) %
            g = [g; -vh(i)-a1*vh(i-1)-a2*vh_(1)-a3*vh_(2)-a4*vh_(3)+...
                           b1*v(i-1,M)+b2*vM_(1)+b3*vM_(2)+b4*vM_(3)]; % ARX model nominal
            lbg = [lbg;0];
            ubg = [ubg;0];
        end      
        if(i==3) %
            g = [g;-vh(i)-a1*vh(i-1)-a2*vh(i-2)-a3*vh_(1)-a4*vh_(2)+...
                          b1*v(i-1,M)+b2*v(i-2,M)+b3*vM_(1)+b4*vM_(2)]; % ARX model nominal
            lbg = [lbg;0];
            ubg = [ubg;0];
        end
        if(i==4) %
            g = [g;-vh(i)-a1*vh(i-1)-a2*vh(i-2)-a3*vh(i-3)-a4*vh_(1)+...
                          b1*v(i-1,M)+b2*v(i-2,M)+b3*v(i-3,M)+b4*vM_(1)]; % ARX model nominal
            lbg = [lbg;0];
            ubg = [ubg;0];
        end
        % after the first 4 time steps, no stored history needed
        if(i>4)
            g = [g;-vh(i)-a1*vh(i-1)-a2*vh(i-2)-a3*vh(i-3)-a4*vh(i-4)+...
                          b1*v(i-1,M)+b2*v(i-2,M)+b3*v(i-3,M)+b4*v(i-4,M)]; % ARX model nominal
            lbg = [lbg;0];
            ubg = [ubg;0];
        end

        % HV position
        if(i==1)
            %ph(i) = p0_h;
        else
            g = [g; -ph(i) + ph(i-1) + dt*vh(i-1)];
            lbg = [lbg;0];
            ubg = [ubg;0];
        end
        % safety constraint on human car and Mth AV
        g = [g;p(i,M)-ph(i)];
        lbg = [lbg;delta];
        ubg = [ubg;10^6];

    end

    % init variables and limit, contains initialization
    lbp = [p0;-10^6*ones(size(p,1)-1,size(p,2))]; % vertcat p0 (of size 1*M) 
    ubp = [p0;+10^6*ones(size(p,1)-1,size(p,2))]; % and N*M size of -INF and +INF for lower and upper bound for AV positions 
    lbv = [v0;v_min*ones(size(v,1)-1,size(v,2))]; % similar to position, for velocity
    ubv = [v0;v_max*ones(size(v,1)-1,size(v,2))];
    lba = a_min*ones(size(a)); % control bounds, no need for initializations
    uba = a_max*ones(size(a));
    
    % constraints for human vehicle init
    lbvh = [v0_h;-10^6*ones(size(vh,1)-1,size(vh,2))]; % ARX velocity
    ubvh = [v0_h;+10^6*ones(size(vh,1)-1,size(vh,2))]; 
    lbph = [p0_h;-10^6*ones(size(ph,1)-1,size(ph,2))]; % ARX + GP positions
    ubph = [p0_h;+10^6*ones(size(ph,1)-1,size(ph,2))]; 
    

    % make variable x and its constraints
    x = [p(:); v(:); a(:); ph(:); vh(:); vh_(:); vM_(:)];
    
    lbx = [lbp(:); lbv(:); lba(:); lbph(:); lbvh(:); lbvh_(:); lbvM_(:)];
    ubx = [ubp(:); ubv(:); uba(:); ubph(:); ubvh(:); ubvh_(:); ubvM_(:)];

    % Get to solver
    options = struct('ipopt', struct('tol', 1e-4, 'acceptable_tol', 1e-2, 'max_iter', 200, 'linear_solver', 'mumps','hessian_approximation','limited-memory',...
        'print_level',5)); %mumps, limited-memory
    options.print_time = true;
    
    prob = struct('f', J, 'x', x, 'g', g); % problem structure
    solver = nlpsol('solver','ipopt',prob,options); %set solver
%     disp('Solving...');

    sol = solver('lbx', lbx, 'ubx', ubx,'lbg', lbg, 'ubg', ubg);  

end




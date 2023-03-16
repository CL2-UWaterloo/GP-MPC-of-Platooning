function [sol] = GP_MPC_platoon(dt, N, M, v_ref, delta, Q, R, ...
                            v_min, v_max, a_max, a_min, ...
                            p0, v0, v0_h, p0_h, p0_h_sigma, prob_desired, ...
                            lbvM_, ubvM_, lbvh_, ubvh_, ...
                            mean, sigma)

%     addpath('C:\Users\JWang\Desktop\Dropbox\Dropbox_share\A_Waterloo\Projects\Human_inloop_Platooning\HILPlatooning-main\MPC\casadi-windows-matlabR2016a-v3.5.5')
%     addpath('/home/jiewang/Downloads/casadi-linux-matlabR2014b-v3.5.5')
    addpath('/home/jwang/Docs/Casadi/casadi-linux-matlabR2014b-v3.5.5')
    import casadi.*

%     gp_model = gpCallback('model');

    % ARX difference model parameters
    a1 = -3.0227; a2 = 3.3543; a3 = -1.6329; a4 = 0.3014; 
    b1 = 0.0063; b2 = -0.0303; b3 = 0.0495; b4 = -0.0254;
        
    % variables
    v = MX.sym('v',N+1,M); % vel of AVs
    p = MX.sym('p',N+1,M); % pos of AVs
    a = MX.sym('a',N,M); % accl of AVs 
    
    % HV ARX norminal and ARX norminal + GP 
    vh = MX.sym('vh',N+1,1); 
    vh_gp = MX.sym('vh',N+1,1); 

    ph = MX.sym('ph',N+1,1); 
    ph_sigma = MX.sym('ph_sigma',N+1,1); % HV position variance  

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
%     for i = 1:N+1 % loop over time steps in horizon
%         % lead vehicle tracking cost
%         J = J + Q*(v_ref(i)-v(i,1))^2;
%         % follower tracking costs
%         for m = 2:M
%             J = J + Q*(v(i,m)-v(i,m-1))^2;
%         end
%         
%         if i < N+1
%             % acceleration cost
%             % lead vehicle tracking cost
%             J = J + R*(a(i,1))^2;
%             % follower tracking costs
%             for m = 2:M
%                 J = J + R*(a(i,m))^2;
%             end
%         else
%             J = J;
%         end

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

        % HV ARX norminal + GP 
        if(i==1) % at first time step in horizon
    %         g = [g; -vh_gp + v0_h];
    %             
    %         lbg = [lbg;0];
    %         ubg = [ubg;0];
        end 
        if(i==2) 
%             x_i = [vh(i-1), v(i-1,M)];
%             [gp_pred{1}, gp_pred{2}] = gp_model(x_i);
%             mean = gp_pred{1};
%             sigma = gp_pred{2};
    
            g = [g; -vh_gp(i) + vh(i) + mean(i-1)]; 
            lbg = [lbg;0];
            ubg = [ubg;0];
        end
        if(i==3) 
%             x_i = [vh(i-1), v(i-1,M)];
%             [gp_pred{1}, gp_pred{2}] = gp_model(x_i);
%             mean = gp_pred{1};
%             sigma = gp_pred{2};
    
            g = [g; -vh_gp(i) + vh(i) + mean(i-1)]; 
            lbg = [lbg;0];
            ubg = [ubg;0];
        end
        if(i==4) 
%             x_i = [vh(i-1), v(i-1,M)];
%             [gp_pred{1}, gp_pred{2}] = gp_model(x_i);
%             mean = gp_pred{1};
%             sigma = gp_pred{2};
    
            g = [g; -vh_gp(i) + vh(i) + mean(i-1)]; 
            lbg = [lbg;0];
            ubg = [ubg;0];
        end
        % after the first 4 time steps, no stored history needed
        if(i>4)
%             x_i = [vh(i-1), v(i-1,M)];
%             [gp_pred{1}, gp_pred{2}] = gp_model(x_i);
%             mean = gp_pred{1};
%             sigma = gp_pred{2};
    
            g = [g; -vh_gp(i) + vh(i) + mean(i-1)]; 
            lbg = [lbg;0];
            ubg = [ubg;0];
        end

        % HV position
        if(i==1) % at first time step in horizon
    %         g = [g; -ph(k) + p0_h];
    %         lbg = [lbg;0];
    %         ubg = [ubg;0];
        end 
        if(i==2) %
            g = [g; -ph(i) + ph(i-1) + dt*vh(i-1)];
            lbg = [lbg;0];
            ubg = [ubg;0];
        end
        if(i>2)
%             x_i = [vh(i-2), v(i-2,M)];
%             [gp_pred{1}, gp_pred{2}] = gp_model(x_i);
%             mean = gp_pred{1};
%             sigma = gp_pred{2};
           
            g = [g; -ph(i) + ph(i-1) + dt*vh(i-1) + dt*mean(i-2)];
            lbg = [lbg;0];
            ubg = [ubg;0];
    
            g = [g; -ph_sigma(i) + ph_sigma(i-1) + dt*dt*sigma(i-2)];
            lbg = [lbg;0];
            ubg = [ubg;0];
        
            % tightened distance constraint on HV and Mth AV
            inverse_cdf = norminv(prob_desired);
            tight_sigma = inverse_cdf*sqrt(ph_sigma(i));
            g = [g; p(i,M)-ph(i)-tight_sigma];
            lbg = [lbg; delta];
            ubg = [ubg;10^6];
        end
    
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

    lbvh_gp = [v0_h;-10^6*ones(size(vh_gp,1)-1,size(vh_gp,2))]; % ARX + GP velocity
    ubvh_gp = [v0_h;+10^6*ones(size(vh_gp,1)-1,size(vh_gp,2))]; 

    lbph = [p0_h;-10^6*ones(size(ph,1)-1,size(ph,2))]; % ARX + GP positions
    ubph = [p0_h;+10^6*ones(size(ph,1)-1,size(ph,2))]; 
    lbph_sigma = [p0_h_sigma;0*ones(size(ph_sigma,1)-1,size(ph_sigma,2))]; % ARX + GP position variance
    ubph_sigma = [p0_h_sigma;+10^6*ones(size(ph_sigma,1)-1,size(ph_sigma,2))]; 

    % make variable x and its constraints
    x = [p(:);v(:);a(:);ph(:);vh(:);vh_gp;vh_(:);vM_(:);ph_sigma(:)];
    
    lbx = [lbp(:);lbv(:);lba(:);lbph(:);lbvh(:);lbvh_gp(:);lbvh_(:);lbvM_(:);lbph_sigma(:)];
    ubx = [ubp(:);ubv(:);uba(:);ubph(:);ubvh(:);ubvh_gp(:);ubvh_(:);ubvM_(:);ubph_sigma(:)];
    % Get to solver
    options = struct('ipopt', struct('tol', 1e-4, 'acceptable_tol', 1e-2, 'max_iter', 30, 'linear_solver', 'mumps','hessian_approximation','limited-memory',...
        'print_level',4)); %mumps, limited-memory
%     options.print_time = true;
    
%     ipopt_opts = struct('print_level',0, 'linear_solver', 'ma27');%, 'hessian_approximation', 'limited-memory');
%     options = struct('ipopt', ipopt_opts,'jit', true, 'compiler', 'shell', 'jit_options', struct('compiler', 'gcc','flags', ['-O3']) );

    prob = struct('f', J, 'x', x, 'g', g); % problem structure
    solver = nlpsol('solver','ipopt',prob,options); %set solver
%     disp('Solving...');

    sol = solver('lbx', lbx, 'ubx', ubx,'lbg', lbg, 'ubg', ubg);  

end




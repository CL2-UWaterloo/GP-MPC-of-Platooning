function [t0, p0, v0, v0_h, v0_h_gp, p0_h_gp] = GP_sysmodel(dt, k, t0, p0, vv0, vv0_h, vv0_h_gp, pp0_h_gp, M, accel)

    a = accel(1,:);

    addpath('/home/jwang/Docs/Casadi/casadi-linux-matlabR2014b-v3.5.5')
    import casadi.*

    gp_model = gpCallback('model');

    % AVs
    for m = 1:M
        v0(m) = vv0(k,m) + dt*a(m);
        p0(m) = p0(m) + dt*vv0(k,m);    % the first k=1 for the vector index reason, but it is the initial values of k=0
    end

    % ARX difference model parameters
    a1 = -3.0227; a2 = 3.3543; a3 = -1.6329; a4 = 0.3014; 
    b1 = 0.0063; b2 = -0.0303; b3 = 0.0495; b4 = -0.0254;

    % HV ARX difference model
    if (k==1) 
        v0_h = vv0_h(1);
    end
    if (k==2) 
        v0_h = -a1*vv0_h(1) + b1*vv0(1,M);
    end
    if (k==3) 
        v0_h = -a1*vv0_h(2)-a2*vv0_h(1) + b1*vv0(2,M)+b2*vv0(1,M); 
    end
    if (k==4) 
        v0_h = -a1*vv0_h(3)-a2*vv0_h(2)-a3*vv0_h(1) + b1*vv0(3,M)+b2*vv0(2,M)+b3*vv0(1,M); 
    end
    if(k>4)
        v0_h = -a1*vv0_h(k-1)-a2*vv0_h(k-2)-a3*vv0_h(k-3)-a4*vv0_h(k-4)+...
               b1*vv0(k-1,M)+b2*vv0(k-2,M)+b3*vv0(k-3,M)+b4*vv0(k-4,M);
    end

    % HV ARX norminal + GP 
    if (k==1) 
        v0_h_gp = v0_h;
    end
    if (k==2)      
        xk = [vv0_h_gp(k-1), vv0(k-1,M)];
        [gp_pred{1}, ~] = gp_model(xk);
        mean = gp_pred{1};

        v0_h_gp = v0_h + mean;
    end
    if (k==3) 
        xk = [vv0_h_gp(k-1), vv0(k-1,M)];
        [gp_pred{1}, ~] = gp_model(xk);
        mean = gp_pred{1};

        v0_h_gp = v0_h + mean;
    end
    if (k==4) 
        xk = [vv0_h_gp(k-1), vv0(k-1,M)];
        [gp_pred{1}, ~] = gp_model(xk);
        mean = gp_pred{1};

        v0_h_gp = v0_h + mean;
    end
    if(k>4)
        xk = [vv0_h_gp(k-1), vv0(k-1,M)];
        [gp_pred{1}, ~] = gp_model(xk);
        mean = gp_pred{1};

        v0_h_gp = v0_h + mean;
    end
    
    v0_h_gp = max(v0_h_gp, 0);

    % position updates
    p0_h_gp = pp0_h_gp(k) + dt*v0_h_gp;

%     % nominal + GP position
%     if(k==1) 
%         p0_h_gp = pp0_h_gp(1);
%     end
%     if(k==2) 
%         p0_h_gp = pp0_h_gp(2) + dt*v0_h;
%     end
%     if(k>2)
%         xk = [vv0_h_gp(k-2), vv0(k-2,M)];
%         [gp_pred{1}, ~] = gp_model(xk);
%         mean = gp_pred{1};
% 
%         p0_h_gp = pp0_h_gp(k) + dt*v0_h + dt*mean;
%     end

    t0 = t0 + dt;
end
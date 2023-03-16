classdef gpCallback < casadi.Callback
  properties
    model
  end
  methods
    function self = gpCallback(name)
      self@casadi.Callback();
      construct(self, name, struct('enable_fd', true));
    end

    % Number of inputs and outputs
    function v=get_n_in(self)
      v=1;
    end
    function v=get_n_out(self)
      v=2;
    end
    % Function sparsity
    function v=get_sparsity_in(self, i)
      v=casadi.Sparsity.dense(2, 1);
    end

    % Initialize the object
    function init(self)
      disp('initializing gpCallback')
      gpr = load('gpr_sparse.mat', 'model'); % the name of 'model' need to match the gpr model training name in "GP.m" (model = fitrgp() )
      self.model = gpr.model;
    end

    % Evaluate numerically
    function [gp_pred] = eval(self, arg)
      x = full(arg{1});
      % Transpose x since gp predictor takes row by row, and casadi gives
      % colum by column
      [mean, sigma, ~] = predict(self.model, x');
      %disp('adding gp')
      gp_pred{1} = mean;
      gp_pred{2} = sigma;
    end
  end
end



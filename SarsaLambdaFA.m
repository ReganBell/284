% Sarsa(lambda) with linear function approximation.
% Action selection is epsilon-greedy.
classdef SarsaLambdaFA
  properties
    alpha
    gamma
    epsilon
    lambda
    linearFA
    traces
    nactions
  end

 %  @param actions	the number of action available to the agent.
 %  @param fa_order	Fourier basis order
 %  @param Alpha	learning rate.
 %  @param Gamma	discount factor.
 %  @param Lambda	lambda parameter.
 %  @param Eps		action selection parameter.
  methods
    function obj = SarsaLambdaFA(state_lb, state_ub, nactions, fa_order, alpha, gamma, lambda, epsilon)
      obj.nactions = nactions;
      obj.alpha = alpha;
      obj.gamma = gamma;
      obj.epsilon = epsilon;
      obj.lambda = lambda;

      obj.linearFA = {};
      for j=1:nactions
        obj.linearFA{j} = FourierFA(length(state_ub), state_lb, state_ub, fa_order); % copy one set of BFs for each discrete action
      end
      
      nbfs = obj.linearFA{1}.nterms; 

      % Setup trace vector.
      obj.traces = zeros(nactions,nbfs);
    end
    
    %  Epsilon-greedy action selection, with ties broken randomly.
    %  
    %  @param s	the state at which the agent will act.
    %  @return	the selected action.
    function a = takeAction(obj,s)
    	% Check for exploration step.
      if(rand() < obj.epsilon)
        a = randi(obj.nactions);
        return;
      end
      
      best_value = -inf;
      
      % For each action ...
      for j=1:obj.nactions
        vj = obj.linearFA{j}.valueAt(s);
               
        % If the action is the best so far ...
        if(vj > best_value)
          best_value = vj;
          a = j;
        end
      end
    end

    % 
    %  Perform an update given a (s, a, r, s', a') tuple.
    %  
    %  @param s		the start state.
    %  @param action	the action.
    %  @param s_prime	the next state (null if we have reached the end of the episode).
    %  @param a_prime	the next action (-1 if we have reached the end of the episode).
    %  @param r			the reward.
    %  
    function obj = update(obj, s, a, r, s_prime, a_prime)
    	% Compute temporal difference error
      delta = r - obj.linearFA{a}.valueAt(s);
        
      % If we're not at the end of the episode add in the value of
      % the next state.
      if (~isnan(s_prime))
      	delta = delta + obj.gamma * obj.linearFA{a_prime}.valueAt(s_prime);
      end
      
      % Check for divergence
      if (isnan(delta))
        error('Function approximation divergence (SarsaLambdaFA)');
      end
      
      % Update each basis function
      for j=1:obj.nactions
      	% First decay traces
        for k=1:obj.linearFA{j}.nterms 
        	obj.traces(j,k) = obj.traces(j,k) * obj.gamma * obj.lambda;
        end

        % Then add active set of basis functions to traces.
        if(j == a)
        	phi = obj.linearFA{j}.computeFeatures(s);
        	
          for k=1:obj.linearFA{j}.nterms 
        		obj.traces(j,k) = obj.traces(j,k) + phi(k);
          end
        end
      end

      for j=1:obj.nactions
        % Build weight deltas to add to weights
        w_deltas = zeros(obj.linearFA{j}.nterms,1);
        for k=1:obj.linearFA{j}.nterms 
          w_deltas(k) = obj.alpha * delta * obj.traces(j,k);
        end
        % Update weights
        obj.linearFA{j} = obj.linearFA{j}.addToWeights(w_deltas);            
      end      
    end 

    %  Compute the value of a state (maximizing over actions).
    %  
    %  @param s	the state.
    %  @return	the value of s.
    %  
    function mq = maxQ(obj,s)
      mq = -inf;
      for j=1:obj.nactions
    		mq = max(mq, obj.linearFA{j}.valueAt(s));
      end
    end   
    
    %  Decay alpha by a given factor.
    %  
    %  @param dec the decay factor.
    %  
    function obj = decayAlpha(obj,dec)
      obj.alpha = obj.alpha * dec;
    end
    
    function obj = decayEpsilon(obj,dec)
      obj.epsilon = obj.epsilon * dec;
    end
    
    function obj = resetWeights(obj)
      for j=1:obj.nactions
        obj.linearFA{j} = obj.linearFA{j}.resetWeights();
      end
    end
    
    %  Clear out traces. This must be done at the
    %  end of every episode. 
    %  
    function obj = clearTraces(obj)
      obj.traces = zeros(obj.nactions,obj.linearFA{1}.nterms);
    end   
    
    function obj = clearEpsilon(obj)
      obj.epsilon = 0.0;
    end
  end
end
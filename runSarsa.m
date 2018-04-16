function runSarsa

% TODO: find a learning rate and exploration rate that works well
alpha =  % learning rate
epsilon =  % exploration factor

% these values should work for you..
gamma=1.0; % MDP discount factor
lambda=0.7; % eligibility decay parameter
fourier_order=7; % what order Fourier basis you want to use
 
% create instance of robot
dt = 0.05;
robot = TimeSteppingRigidBodyManipulator(PlanarRigidBodyManipulator('Pendulum.urdf'),dt);
% v= robot.constructVisualizer;

nactions = 3;
torque_max = 3.0;
actions = linspace(-torque_max,torque_max,nactions);

% state upper and lower bounds, since Fourier basis requires states to be
% scaled [0,1]
state_lb = [0,-20];
state_ub = [2*pi, 20];

% create function approximator and learner
sarsa_agent = SarsaLambdaFA(state_lb, state_ub, nactions, fourier_order, alpha, gamma, lambda, epsilon);
 
n_learning_trials = 10; % how many indepedent learning experiments we want to do
n_episodes = 30; % how many episodes per learning experiment
max_iters = 400; % how many steps per episode

% TODO: add code within the following loops to implement learning using Sarsa(\lambda) 

for lt = 1:n_learning_trials

  for ep=1:n_episodes

    for i=1:max_iters


    end

    % Clear traces
    sarsa_agent = sarsa_agent.update(s, a, r, nan, -1);
    sarsa_agent = sarsa_agent.clearTraces();    
  end

% Reset learner
  sarsa_agent = sarsa_agent.resetWeights();
  sarsa_agent = sarsa_agent.clearTraces();   
end

% the reward function. You shouldn't have to change this for the assignment, 
% but you could experiment with other reward functions here if you want. 
function r = reward(s,a)
  sd = [pi;0];
  r = -dt*0.5*(s-sd)'*(s-sd);
end

end
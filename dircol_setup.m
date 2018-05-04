function [p, traj_opt] = dircol_setup(N, T)
    options.replace_cylinders_with_capsules = false;
    options.wrap_flag = [true;false]; % necessary?
    p = PlanarRigidBodyManipulator('Pendulum.urdf',options);
    p = p.setInputLimits(-10,10);
    
    traj_opt = DircolTrajectoryOptimization(p, N, [T/2 T]);

    traj_opt = traj_opt.setSolver('fmincon');
    traj_opt = traj_opt.setSolverOptions('fmincon','Algorithm','sqp');
end
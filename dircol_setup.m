function [p, traj_opt] = dircol_setup(N, T, u)
    options.replace_cylinders_with_capsules = false;
    p = PlanarRigidBodyManipulator('Pendulum.urdf',options);
    p = p.setInputLimits(-u,u);
    
    traj_opt = DircolTrajectoryOptimization(p, N, [T/2 T]);

    traj_opt = traj_opt.setSolver('fmincon');
    traj_opt = traj_opt.setSolverOptions('fmincon','Algorithm','sqp');
end
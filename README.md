# BRUCE_control
BRUCE Locomotion controller based on WBIC, WeightedWBC, and HO-WBC with pinocchio library

It's an implementation of several WBC methods. 

Trajectory Optimization, Angular Momentum-based Footstep Planning, and MPC related planner will be added in future, and will be able to chose which WBC to control the robots. The sources are based on three references(listed paper below), and three original sources (OCS2, legged_control, mit_software). I am not namespace lover, I have deleted some namespace defined at the original sources, which might cause confusion. 

Kinematics & Dynamics were compared between MuJoCo and Pinocchio with tolerance at 1e-6. And, It haven't been yet to implemented on real physical robots. 

The robot model (BRUCE) was provided by WESTWOOD ROBOTICS.
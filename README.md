# Applied-Control-Systems-Module
My attempts at ELEC6228 Applied Control Systems tasks. This consisted of three individual works to show understanding of LQR, MPC and ILC, followed by a hardware implementation group project. Conducted in April 2022.

Group project LabVIEW hardware and software versions: LabVIEW 2018 with Quanser Qube Servo and NI MyRIO. 

Group_Project:

Software simulation of Lego Mindstorms NXTWay (MATLAB).

Software simulations and hardware implementations of LQR, MPC on a rotary inverted pendulum (LabVIEW). To enable reference tracking for MPC, LabVIEW's MPC blocks were used rather than explicit MathScript blocks (to reduce the runtime of each control step). Despite MPC simulating well with a prediction horizon of 8-11, it is worth noting that the practical version of MPC required (in our case) a prediction horizon of 120-150.

Software simulation of ILC on an inertial disc state-space model (LabVIEW).

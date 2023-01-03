# Applied-Control-Systems-Module
My attempts at ELEC6228 Applied Control Systems tasks. This consisted of three individual works to show understanding of LQR, MPC and ILC, followed by a hardware implementation group project. 

Group project LabVIEW hardware and software versions: LabVIEW 2018 with Quanser Qube Servo and NI MyRIO. 

Group_Project:



Software simulation of Lego Mindstorms NXTWay (MATLAB).

Hardware implementations of LQR, MPC on an inverted pendulum (LabVIEW). To enable reference tracking for MPC, LabVIEW's MPC blocks were used rather than explicit MathScript blocks (to reduce the runtime of each control step). Despite simulating well with a prediction horizon of 8, it is worth noting that the practical version of MPC required (in our case) a prediction horizon of 120-150.

Software simulation of ILC on an inverted pedulum state-space model (LabVIEW).

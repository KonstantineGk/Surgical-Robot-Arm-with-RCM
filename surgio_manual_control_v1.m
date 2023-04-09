%------- TEAM PROJECT - SURGICAL ROBOTICS -------%
%------- 1066600 --------- 1067480 --------------%
clc; clear; close all;

% DH parameters
a = [ 0, 0, 0, 0, 0, 0,0 ];
alpha= deg2rad([ 90,-90, 90, -90,90,-90, 0]);
d =  [ 0.124, 0, 0.14, 0, 0.082, 0, 0.18] ;

% Robot limits
ql = deg2rad([-90,90; -45,90; -135,45; -120,120; -135,125; -30,130; -135,125]);
N=7;

% Create robot's Links
for i = 1:7
    L(i) = Link( [0 d(i) a(i) alpha(i)] ) ;
end

% Generate the object robot
robot = SerialLink(L);

% Insert servo limitations to the robotic model
robot.qlim = ql;
robot.name="SURGIO RAMIS";

%------------- Starting deg ------------------------------------------%
q = deg2rad([0 80.1 0 80.1 0 0 0]); % Working
%q = deg2rad([0 80 20 -80 0 -20 0]); % Working
%q = deg2rad([0 80 20 -80 0 0 0]); % Working
%q = [0.9887    1.3488   -1.9573    1.7315    0.5134   -0.2512   -1.0924]; Working
%q = [-1.4586    1.2153    0.5780    0.7487    1.0823    1.5516   -0.5763];
%q = [-0.1801   -0.5341    0.6657   -2.0750    1.1602    1.7587    1.5858];
%q = spot_testing();
%------------- Calculate Pc(RCM point) along the last link ----------------------%
[T,Ps,Rt] = forward_kine(q,alpha,[ 0.124, 0, 0.14, 0, 0.082, 0, 0.10]);
Pc = Ps;
%--------- Tooltip Velocity ------%
Vu = [0 1 0 0 0];
Vt = [0 0 0 0 0 0];
%---------------------------------%
dt = 1e-2;
t=0;
i=1;
%----------------------------Plot-------------------------------%
figure("Name","SIMULATION","NumberTitle","off");
robot.plot(q,'trail', 'k--','delay',dt,'lightpos', [1 1 1]);
%------------------ Connect Joystick -----------------------%
joy = vrjoystick(1);
%--------------------%
 while true
    %----------Joystick Control------------%
    buttonExit = button(joy,1);
    axisx = -0.3 * round(axis(joy,1));
    axisy = -0.5 * round(axis(joy,2));
    axisz =  0.5 * round(axis(joy,3));
    rotate_right = round(button(joy,6));
    rotate_left = -round(button(joy,5));
    if buttonExit == 1
        disp("EXITING ...")
        close figure 1;
        break
    end
    Vu = [axisz axisx axisy rotate_left+rotate_right 0];
    if(Vu == [0 0 0 0 0])Vt = [0 0 0 0 0 0];end
    %-----------------------------------------------------------%
    [T,Pt,Rt] = forward_kine(q(i,:),alpha,[ 0.124, 0, 0.14, 0, 0.082, 0, 0.18]); % Forward Kinematics
    Nt = Rt(1:3,3); % Z Axis
    Bc = Rt(1:3,1:2); % X Y Axis
    Jac = robot.jacob0(q(i,:)); % Jacobian in tool orientation
    %-----------------------------------------------------------%
    % Ax is the RCM constraint Jacobian in tool tip operational space
    Ax = Bc' * [eye(3,3), skew(Pt - Pc)];
    %Matrix A denotes the RCM constraint Jacobian in the joint space.
    A = Ax * Jac;
    % Zx is the basis for the null space of Ax
    Zx = [Nt', zeros(1,3); Rt' * skew(Pt - Pc), Rt'];
    Z = (pinv(Jac) * Zx')';
    % G is the base matrix of the null space of the Jacobian
    G = null(Jac)';
    % N is the basis of the null space of the RCM constraint Jacobian
    N = ([Z' G'])';
    % Matrix S is a full rank matrix assuming
    % a motion away from kinematics singularities
    S = [ pinv(A), N'];
    % Calculate Qdot using previous velocities
    qdot = pinv(Jac) * Vt' ;
    % Calculate Xcdot
    Xcdot = A * qdot ;
    % Extracting the new velocities
    Vt = ([Zx' , zeros(6,1)] * Vu' + Jac*pinv(A)*Xcdot)';
    % Adjust Qdot using Xcdot
    qdot = S * [Xcdot ; Vu'];
    % Saving the new angles
    q(i+1,:) = q(i,:) + dt * qdot';
    %----------- Error ---------------%
    Xc = Bc'*(Pt-Pc);
    error(i) = round(norm(Xc),5);
    if(error(i)>0.03)
            q(i+1,:) = q(1,:);
            Vt = [0 0 0 0 0 0];
    end
    %----Check for singularities------%
    Jac = robot.jacob0(q(i+1,:));
    omega = sqrt(det(Jac * Jac'));
    if(omega<0.0008)
            q(i+1,:) = q(i,:);
    end
    %------ Check robot limits -------%
    check = robot.islimit(q(i+1,:));
    if norm(check(:,1)) >=1
            q(i+1,:) = q(i,:);
    end
    %---------------------------------%
    t = t+dt;
    i = i+1;
    robot.animate(q(i,:));
 end
 plot(error);
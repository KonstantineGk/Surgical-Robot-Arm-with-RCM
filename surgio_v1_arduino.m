%------- TEAM PROJECT - SURGICAL ROBOTICS -------%
%------- 1066600 --------- 1067480 --------------%
clear all; close all; clc;
% construct the object arduino
a = arduino('COM5');
% num of servos
n = 7;
s1 = servo(a, 'D3',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
s2 = servo(a, 'D4',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
s3 = servo(a, 'D5',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
s4 = servo(a, 'D6',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
s5 = servo(a, 'D7',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
s6 = servo(a, 'D8',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
s7 = servo(a, 'D9',  'MinPulseDuration', 500*10^-6, 'MaxPulseDuration', 2600*10^-6);
s = [s1;s2;s3;s4;s5;s6;s7];
pause(1);

% DH parameters
a = [ 0, 0, 0, 0, 0, 0,0 ];
alpha= deg2rad([ 90, -90, 90, -90,90,-90, 0]);
d =  [ 0.124, 0, 0.14, 0, 0.08, 0, 0.29] ;

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
% 1 % q = deg2rad([0 40 20 -80 0 -20 0]); % Working
% 2 % q = [0.1473    0.9582    0.6751   -1.4342    2.0482    2.1493   -0.1536];
%------------- Calculate Pc(RCM point) along the last link ----------------------%
[T,Ps,Rt] = forward_kine(q,alpha,[ 0.124, 0, 0.14, 0, 0.08, 0, 0.26]);
Pc = Ps;
%--------- Tooltip Velocity ------%
Vu = [0 0 0.15 1 0];
Vt = [0 0 0 0 0 0];
%---------------------------------%
dt = 5e-3;
t=0;
i=1;
%----------------------------Plot-------------------------------%
%   figure("Name","SIMULATION","NumberTitle","off");
%  robot.plot(q,'trail', 'k--','delay',dt,'lightpos', [1 1 1]);
 while t<20
    %-----------------------------------------------------------%
    [T,Pt,Rt] = forward_kine(q(i,:),alpha,[ 0.124, 0, 0.14, 0, 0.08, 0, 0.29]); % Forward Kinematics
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
    %----------------------- Error -----------------------------%
%     Xc = Bc'*(Pt-Pc);
%     error(i) = round(norm(Xc),5);
%     if(error(i)>0.01)
%             q(i+1,:) = q(1,:);
%             Vt = [0 0 0 0 0 0]; % Reverse Speed
%      end
    %----Check for singularities------%
    Jac = robot.jacob0(q(i,:));
    omega = sqrt(det(Jac * Jac'));
    if(omega<0.0008)
            q(i+1,:) = q(i-1,:);
            Vt = -Vt; % Reverse Speed
            Vu = -Vu; % Reverse Speed
    end
    %------ Check robot limits -------%
    check = robot.islimit(q(i+1,:));
    if norm(check(:,1)) >=1
            q(i+1,:) = q(i,:);
            Vt = -Vt; % Reverse Speed
            Vu = -Vu; % Reverse Speed
    end
    %---------------------------------%
    t = t+dt;
    i = i+1;
 end
 
%use this after having calculated the joint angles q to move the servos
% dt is the Euler integration step you chose (di corresponds to the sampling step - you might have to change it)
di = 0.10/dt;
%convert q from rads to degs
theta = round(rad2deg(q));
dq = [0 -6 0 -4 0 -7 0];
for i=1:length(theta)
    theta(i,:) = theta(i,:) + dq(1,:);
    for j=1:n
        if j==4
            theta(i,j)=-theta(i,j);
        end
    end
end
% maxiter is the size of euler iterations
angle = zeros(length(q),n);
%normalization to range [0,0.95]
for j=1:n
        angle(:,j) = 0.95*((theta(:,j) + 140 ) / (270));   
end
% write the normalized angles to each servo
for i =1:di:length(q)
    for j=1:n
            %update the position of servos
             writePosition(s(j), angle(i,j));
    end
end

%----------------------------Plot-------------------------------%
  figure("Name","SIMULATION","NumberTitle","off");
  robot.plot(q(1:di:end,:),'trail', 'k--','lightpos', [1 1 1]);
  
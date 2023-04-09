function [T,Position,Rotation] = forward_kine(theta,alpha,d)
    T = eye(4,4);
    pr = 4;
    %----------------- Calculate Forward Kinematics using D-H parameters -%
    for i = 1:7
          A = [ round(cos(theta(i)),pr)         -round(sin(theta(i))*cos(alpha(i)),pr)   round(sin(theta(i))*sin(alpha(i)),pr)    0;
                round(sin(theta(i)),pr)         round(cos(theta(i))*cos(alpha(i)),pr)   -round(cos(theta(i))*sin(alpha(i)),pr)    0;  
                0                           round(sin(alpha(i)),pr)              round(cos(alpha(i)),pr)              d(i);
                0                                     0                         0                             1;
                ];
          T = T*A;
          Position = T(1:3,4);
          Rotation = T(1:3,1:3);
    end
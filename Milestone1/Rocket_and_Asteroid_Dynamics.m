%% Author: _Luke Baatjes_
% _EEE4119F Mechatronics II Project Milestone 1_
% Date: _05/03/2023_
% 
% 
% 
%% *ROCKET MODEL*
%% _Symbolic Variables and Generalized Coordinates_

syms x dx ddx y dy ddy th dth ddth 'real'

% ROCKET PARAMETERS
syms alph F m d g Iz

% The rocket is allowed to translate in the x and y directions and allowed
% to rotate about an angle theta in the z axis of rotation.
% GENERALISED COORDINATES
q = [x; y; th];
dq = [dx; dy; dth];
ddq = [ddx; ddy; ddth];
%% _Rotation Into, Positions, and Velocities in Inertial Frame_

% ROTATIONS
% There is only 1 other frame relative to the Inertial frame hence the need
% for only one rotation about the Z axis
R01 = Rotz(th);                 % Rotation from Inertial Frame to Body Frame
R10 = transpose(R01);           % Rotation from Body Frame to Inertial Frame

% POSITIONS
rR_0 = [x;y;0];                 % Rocket's center of mass Position in Inertial Frame
rF_B = [0;-d;0];                % Position of Force F located at position -d in body Frame
rF_0 = rR_0 + R10*rF_B;         % Position of Force F in Inertial Frame (Sum of position of centre of mass of rocket and position of force rotated to the Inertial frame)

% VELOCITIES
drR_0 = jacobian(rR_0,q)*dq;    % Translational velocity of rocket
wR_0 = dth;                     % Angular velocity of the rocket

%% _Energy of Rocket_

% KINETIC ENERGY
T1 = 0.5*m*transpose(drR_0)*drR_0;     % Translational Kinetic Energy
T2 = 0.5*transpose(wR_0)*Iz*wR_0;      % Angular Kinetic Energy. In this case the Mass Moment of Inertia is calculated using the parallel axis theorem (1/12*m*(L1^2 + L2^2) + md^2).
Ttot = simplify(T1 + T2);

% POTENTIAL ENERGY
Vtot = simplify(m*g*rR_0(2));
%% _Define Mass Matrix_

% MASS MATRIX
M = hessian(Ttot,dq);
%% _Define Mass Matrix Deriv_

dM = sym(zeros(length(M),length(M)));
for i=1:length(M)
    for j=1:length(M)
        dM(i,j) = jacobian(M(i,j),q)*dq;
    end
end
dM = simplify(dM);
%% _Define Gravity Matrix_

% GRAVITY MATRIX
G = jacobian(Vtot,q);
G = simplify(G);
%% _Define Coriolis Matrix_

% CORIOLIS MATRIX
C = dM*dq - transpose(jacobian(Ttot,q));
C = simplify(C);
%% _Force Position and Generalised Forces_

% FORCES
% Magnitude of force in Inertial Frame
Fvec_B = [-F*sin(alph);F*cos(alph);0];              % Force in Frame 1, using trigonometry to resolve force into components in the body frame
Fvec_0 = simplify(R10*Fvec_B);                      % Force in Inertial frame

% GENERALIZED FORCES
% Partial derivatives of position where force acts
partX = jacobian(rF_0,x);
partY = jacobian(rF_0,y);
partTh = jacobian(rF_0,th);

% Generalised force components
Qx = simplify(Fvec_0(1)*partX(1) + Fvec_0(2)*partX(2) + Fvec_0(3)*partX(3));
Qy = simplify(Fvec_0(1)*partY(1) + Fvec_0(2)*partY(2) + Fvec_0(3)*partY(3));
Qth = simplify(Fvec_0(1)*partTh(1) + Fvec_0(2)*partTh(2) + Fvec_0(3)*partTh(3));

Q = [Qx; Qy; Qth];
%% _Define Manipulator Equation_

% MANIPULATOR EQUATION
ManipulatorEqn = M*ddq + C + G.' == Q;
    
% Solve for accelerations
ddx = simplify(solve(ManipulatorEqn(1), ddx));
ddy = simplify(solve(ManipulatorEqn(2), ddy));
ddth = simplify(solve(ManipulatorEqn(3), ddth));
%% _Rocket Accelerations in the X, Y, and Theta Directions_

disp("Rocket Accelerations:")
acceleration = [ddx;ddy;ddth]
%% *ASTEROID MODEL*
% 
%% _Determining the Asteroids Drag Coefficient Value_

% System accepts velocities in the x, y, and theta directions and is
% expected to output the accelerations of the Asteroid in simulation
% Run Simulation to populate workspace
sim('AsteroidImpact');

% Determine Accelerations in the x and y directions by taking the time
% derivitive of the input velocities
ast_ddx = diff(ast_dx)./diff(simulation_time);
ast_ddy = diff(ast_dy)./diff(simulation_time);

plot(ast_ddx, "DisplayName","ast dx")
xlabel("Simulation Time")
ylabel("Filtered Acceleration in x Direction")
hold on

% Using the acceleration of the asteroid in the x direction as an example,
% it is clear that using a regular filter results in a poor reduction of
% noise (Orange curve)
ddx_ast_filter = smoothdata(ast_ddx);

% Filter the moise accelerations by using a Guassian-Weighted Moving
% Average Filter with a window size of 20
% It is clear that a gaussian filter produces a much better result (Yellow curve)
ddx_ast_filter1 = smoothdata(ast_ddx,"gaussian",20);
ddy_ast_filter1 = smoothdata(ast_ddy,"gaussian",20);

plot(ddx_ast_filter, "DisplayName","smooth filter")
plot(ddx_ast_filter1, "DisplayName","gaussian filter")
legend
hold off

% Determine drag force in the X direction
Fdrag_x = 10000*ddx_ast_filter1;

% Determine drag force in the Y direction
g_array = 9.81*ones(length(ddy_ast_filter1));
Fdrag_y = 10000*(ddy_ast_filter1 + g_array);

% Determine the magnitude of the total drag force by using pythagoras
% theorem
Fdrag = sqrt(Fdrag_x.^2 + Fdrag_y.^2);

% Determine the magnitude of the total velocity using pythagoras theorem
ast_dx_filter = smoothdata(ast_dx, "gaussian", 20);
ast_dy_filter = smoothdata(ast_dy, "gaussian", 20);
dq = sqrt(ast_dx_filter.^2 + ast_dy_filter.^2);

% Determine array of drag coefficients
c_array = Fdrag./dq(1:end-1);

% Final drag coefficient value
c = mean(c_array);
disp("Drag force coefficient c:")
disp(c(1))
%% _Rotation Function_

function A = Rotz(th)
    A = [cos(th)   sin(th) 0;... 
         -sin(th)  cos(th) 0;...
         0        0        1];
end

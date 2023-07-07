%% make an animation
close all;
define_constants;

% look at the source of make_animation if it doesn't work - you may need to
% lower the fps on slower computers. Otherwise you can uncomment a line to
% make the animation happen faster than real time
set_param(['AsteroidImpact', '/rngSeed'], 'Value', '736');
sim('AsteroidImpact');  % tell simulink to simulate the model
make_animation(simulation_time, x, y, th, ast_x, ast_y, ast_th, ast_dx, ast_dy, 10)

% 'simulation_time' is an array of time values, exported from the simulink
% simulation

%% Make a video animation and save it to rocket-animation.avi
% this is more useful if the animation script above gives you problems
% % or if you want to share a video with anyone else
% record_animation(simulation_time, x, y, th, ast_x, ast_y, ast_th, ast_dx, ast_dy)

%% check whether you succeeded
Scenario = 1;
mission_complete(x, y, ast_x, ast_y, ast_th, Scenario)

%% determine x distance between the city and rocket-asteroid intercept and the distance between the rocket and asteroid at detonation
disp('---------------------------------------------')
xrange = 2000 - ast_x(end);
xrange1 = abs(x(end) - ast_x(end));
yrange1 = abs(y(end) - ast_y(end));
dist = xrange;
dist_from_city = ['x distance between city and rocket-asteroid intercept: ', num2str(dist), 'm'];
disp(dist_from_city)
dist1 = sqrt(xrange1^2 + yrange1^2);
dist_betw_rocket_and_ast = ['Distance between rocket and asteroid: ', num2str(dist1), 'm'];
disp(dist_betw_rocket_and_ast)
disp('---------------------------------------------')
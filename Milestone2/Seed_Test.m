%% _SEED TEST_
%% ***_PLEASE NOTE THAT YOU HAVE TO OPEN THE AsteroidImpact.slx FILE BEFORE RUNNING THIS SCRIPT_***
close all;
clc;
%
% This script will run the simulation a specified number of times (number
% of times run is set by 'runs' variable below), each time with a randomly
% generated seed value in the range of 100 to 500. This is to show the
% robustness of the controller.
% In the command window you will be able to see both the x distance between
% the rocket-asteroid intercept and city and the distance
% between the rocket and asteroid when the rocket detonates

Scenario = 1; % This doesnt really matter for scenario 1 and 2

HIT = 0;
runs = 10;
seed = randi([100 1000], 1, runs);

for i = 1:runs
    set_param(['AsteroidImpact', '/rngSeed'], 'Value', 'seed(i)');
    sim('AsteroidImpact.slx');  % tell simulink to simulate the model

    if (mission_complete(x, y, ast_x, ast_y, ast_th, Scenario)) == true

        % Plot trajectories and interception
        plot(ast_x,ast_y)
        hold on
        plot(x,y)

        HIT = HIT + 1;
        xrange = 2000 - ast_x(end);
        xrange1 = abs(x(end) - ast_x(end));
        yrange1 = abs(y(end) - ast_y(end));
        dist = xrange;
        run_instance = ['Seed number ', num2str(i)];
        disp(run_instance);
        seed_value = ['Seed value: ', num2str(seed(i))];
        disp(seed_value)
        dist_from_city = ['x distance between city and rocket-asteroid intercept: ', num2str(dist), 'm'];
        disp(dist_from_city)
        dist1 = sqrt(xrange1^2 + yrange1^2);
        dist_betw_rocket_and_ast = ['Distance between rocket and asteroid: ', num2str(dist1), 'm'];
        disp(dist_betw_rocket_and_ast)
        disp('---------------------------------------------')
    else
        mission_complete(x, y, ast_x, ast_y, ast_th, Scenario)
        HIT = HIT;
    end

end

accuracy = (HIT/runs)*100;
acc = ['Accuracy: ', num2str(accuracy)];
disp(acc)
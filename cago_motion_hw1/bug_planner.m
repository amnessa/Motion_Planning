function [x,y] = bug_planner( qstart, qgoal )

global sensor_range infinity;

% A simple hack which blindly goes towards the goal
samples = norm( qgoal - qstart ) / (sensor_range / 10);
range = linspace(0,1,samples);

x = qstart(1) + range*(qgoal(1) - qstart(1));
y = qstart(2) + range*(qgoal(2) - qstart(2));

end

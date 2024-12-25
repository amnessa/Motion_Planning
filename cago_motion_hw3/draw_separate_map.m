% Define obstacle configuration
O = {[5 5 10 10 ;-5 17 17 -5], [5 5 10 10;18 35 35 18 ]};


% Create a figure
figure;
hold on;

% Set axis limits
axis([0 30 0 30]);
axis equal; % Keep the aspect ratio square

% Plot each obstacle
for i = 1:length(O)
    obstacle = O{i}; % Extract the obstacle vertices
    patch(obstacle(1, :), obstacle(2, :), 'yellow', 'FaceAlpha', 0.5, 'EdgeColor', 'black'); % Plot with transparency
end

% Add labels
title('Obstacle Configuration');
xlabel('X');
ylabel('Y');

% Display the grid for better visualization
grid on;

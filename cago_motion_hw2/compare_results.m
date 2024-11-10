% compare_results.m - Function to compare both approaches
function compare_results(results_potential, results_navigation)
    figure;
    subplot(2,1,1);
    plot(results_potential.time, vecnorm(results_potential.path, 2, 2));
    title('Distance to Goal - Potential Field');
    xlabel('Time');
    ylabel('Distance');
    
    subplot(2,1,2);
    plot(results_navigation.time, vecnorm(results_navigation.path, 2, 2));
    title('Distance to Goal - Navigation Function');
    xlabel('Time');
    ylabel('Distance');
end
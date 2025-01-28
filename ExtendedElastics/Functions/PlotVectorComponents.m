function PlotVectorComponents(ResVec)

    figure
    subplot(3,1,1)
    grid on
    plot(ResVec(1,:))
    ylabel('1st-comp.')

    subplot(3,1,2)
    grid on
    plot(ResVec(2,:))
    ylabel('2nd-comp.')

    subplot(3,1,3)
    grid on
    plot(ResVec(3,:))
    ylabel('3rd-comp.')

    fprintf('Mean value of 1st component: %6.4e \n', mean(ResVec(1,:)))
    fprintf('Max value of 1st component: %6.4e \n', max(ResVec(1,:)))
    fprintf('Min value of 1st component: %6.4e \n', min(ResVec(1,:)))
    fprintf('The steady state value of the 1st component is: %6.4e \n', mean(ResVec(1,1:3000)))

    fprintf('Mean value of 2nd component: %6.4e \n', mean(ResVec(2,:)))
    fprintf('Max value of 2nd component: %6.4e \n', max(ResVec(2,:)))
    fprintf('Min value of 2nd component: %6.4e \n', min(ResVec(2,:)))
    fprintf('The steady state value of the 2nd component is: %6.4e \n', mean(ResVec(2,1:3000)))

    fprintf('Mean value of 3rd component: %6.4e \n', mean(ResVec(3,:)))
    fprintf('Max value of 3rd component: %6.4e \n', max(ResVec(3,:)))
    fprintf('Min value of 3rd component: %6.4e \n', min(ResVec(3,:)))
    fprintf('The steady state value of the 3rd component is: %6.4e \n', mean(ResVec(3,1:3000)))

end
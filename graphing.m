function graphing(logsout)
    figure
    hold on
    sgtitle('States of the System (Nonlinear)')
    subplot(4,3,1)
    plot(logsout{12}.Values.Time',logsout{12}.Values.Data)
    xlabel('Time (sec)')
    ylabel('x_{e} [m]')
    grid
    subplot(4,3,2)
    plot(logsout{14}.Values.Time',logsout{14}.Values.Data)
    xlabel('Time (sec)')
    ylabel('y_{e} [m]')
    ylim([-2 2])
    grid
    subplot(4,3,3)
    plot(logsout{16}.Values.Time',logsout{16}.Values.Data)
    xlabel('Time (sec)')
    ylabel('z_{e} [m]')
    ylim([-1 1])
    grid
    subplot(4,3,4)
    plot(logsout{11}.Values.Time',logsout{11}.Values.Data)
    xlabel('Time (sec)')
    ylabel('x_{dot} [m/s]')
    grid
    subplot(4,3,5)
    plot(logsout{13}.Values.Time',logsout{13}.Values.Data)
    xlabel('Time (sec)')
    ylabel('y_{dot} [m/s]')
    ylim([-1 1])
    grid
    subplot(4,3,6)
    plot(logsout{15}.Values.Time',logsout{15}.Values.Data)
    xlabel('Time (sec)')
    ylabel('z_{dot} [m/s]')
    ylim([-1 1])
    grid
    subplot(4,3,7)
    plot(logsout{5}.Values.Time',logsout{5}.Values.Data)
    xlabel('Time (sec)')
    ylabel('p [rad/s]')
    ylim([-1 1])
    grid
    subplot(4,3,8)
    plot(logsout{8}.Values.Time',logsout{8}.Values.Data)
    xlabel('Time (sec)')
    ylabel('q [rad/s]')
    grid
    subplot(4,3,9)
    plot(logsout{9}.Values.Time',logsout{9}.Values.Data)
    xlabel('Time (sec)')
    ylabel('r [rad/s]')
    ylim([-1 1])
    grid
    subplot(4,3,10)
    plot(logsout{6}.Values.Time',logsout{6}.Values.Data)
    xlabel('Time (sec)')
    ylabel('phi [rads]')
    ylim([-1 1])
    grid
    subplot(4,3,11)
    plot(logsout{10}.Values.Time',logsout{10}.Values.Data)
    xlabel('Time (sec)')
    ylabel('theta [rads]')
    grid
    subplot(4,3,12)
    plot(logsout{7}.Values.Time',logsout{7}.Values.Data)
    xlabel('Time (sec)')
    ylabel('psi [rads]')
    ylim([-1 1])
    grid
    hold off

    figure
    hold on
    sgtitle('Inputs of the System')
    subplot(2,2,1)
    plot(logsout{1}.Values.Time',logsout{1}.Values.Data)
    xlabel('Time (sec)')
    ylabel('F1 [N]')
    grid
    subplot(2,2,2)
    plot(logsout{2}.Values.Time',logsout{2}.Values.Data)
    xlabel('Time (sec)')
    ylabel('F2 [N]')
    grid
    subplot(2,2,3)
    plot(logsout{3}.Values.Time',logsout{3}.Values.Data)
    xlabel('Time (sec)')
    ylabel('F3 [N]')
    grid
    subplot(2,2,4)
    plot(logsout{4}.Values.Time',logsout{4}.Values.Data)
    xlabel('Time (sec)')
    ylabel('F4 [N]')
    grid
    hold off
end
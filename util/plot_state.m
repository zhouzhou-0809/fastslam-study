function plot_state(SLAM, gt, trajectory, landmarks, timestep, z, window)
    % Visualizes the state of the FastSLAM algorithm.
    %
    % The resulting plot displays the following information:
    % - map ground truth (black +'s)
    % - currently best particle (red)
    % - particle set in green
    % - current landmark pose estimates (blue)
    % - visualization of the observations made at this time step (line between robot and landmark)
    clf; global err;
    hold on
    grid("on"); 
    %graphics_toolkit gnuplot
    L = struct2cell(landmarks);
    alpha   = 0.5;
    plot(cell2mat(L(2,:)), cell2mat(L(3,:)), 'o', 'color', [0,0,0] + alpha, 'markersize', 15, 'linewidth', 2);
    text(cell2mat(L(2,:)), cell2mat(L(3,:)), string(cell2mat(L(1,:))), 'FontSize', 8);

    % Plot the particles
    ppos = [SLAM.particle.pose];
    plot(ppos(1,:), ppos(2,:), 'g.');

    % determine the currently best particle
    [~, bestParticleIdx] = max([SLAM.particle.weight]);

    % draw the landmark locations along with the ellipsoids
    % Plot for FastSLAM with known data association
    is_known    = isfield(SLAM.particle(bestParticleIdx).landmark, 'isobserved'); % is data association known?
    if is_known
        for i = 1:length(SLAM.particle(bestParticleIdx).landmark)
            if SLAM.particle(bestParticleIdx).landmark(i).isobserved
                l = SLAM.particle(bestParticleIdx).landmark(i).EKF.mu;
                plot(l(1), l(2), 'bo', 'markersize', 3);
                drawprobellipse(l, SLAM.particle(bestParticleIdx).landmark(i).EKF.Sigma, 0.95, 'b');
                
                % Extract landmarks estimation error
                err.mean(i,timestep)    = norm(l-[landmarks(i).x; landmarks(i).y],2);
                err.sig(i,timestep)     = err.sig(i,timestep) + sqrt(norm(SLAM.particle(bestParticleIdx).landmark(i).EKF.Sigma));
            end
        end
    % Plot for FastSLAM with unknown data association
    else
        X = []; % store Landmarks position of best particle
        for i = 1:length(SLAM.particle(bestParticleIdx).landmark)
            l = SLAM.particle(bestParticleIdx).landmark(i).EKF.mu;
            X = [X,l]; % store Landmarks position of best particle
            plot(l(1), l(2), 'bo', 'markersize', 3);
            drawprobellipse(l, SLAM.particle(bestParticleIdx).landmark(i).EKF.Sigma, 0.95, 'b');
        end
        % Extract landmarks estimation error
        obsv_ID     = [z.id]; % Observed landmarks at this timestep
        if timestep == 1
            err.obsv_ID(timestep)    = {obsv_ID}; % Initial observed landmarks
        else
            % All observed landmarks' ID by now
            err.obsv_ID(timestep)    = {union(cell2mat(err.obsv_ID(timestep-1)), obsv_ID)};
        end
        err.obsv_N(timestep)    = length(cell2mat(err.obsv_ID(timestep)));
        err.store(timestep)     = length(SLAM.particle(bestParticleIdx).landmark);
        % Logrithm err calculation for best association I
        IDs = cell2mat(err.obsv_ID(timestep));
        Mx  = [landmarks(IDs).x];
        My  = [landmarks(IDs).y];
        M   = [Mx; My];
        I   = pinv(X)*M;
        [~,Err,~]   = svd(I,'econ');
        err.mean(timestep)  = norm(logm(Err)) / err.obsv_N(timestep);
    end
    

    % draw the observations
    for i = 1:length(z)
        pose    = SLAM.particle(bestParticleIdx).pose;
        l_x     = pose(1) + z(i).range*cos(pose(3)+z(i).bearing);
        l_y     = pose(2) + z(i).range*sin(pose(3)+z(i).bearing);
        line([pose(1), l_x], [pose(2), l_y],...
            'color', 'r', 'LineStyle','--', 'linewidth', 1);
    end
        
    % draw the groud true trajectory
    line(gt(1,1:timestep), gt(2,1:timestep), 'color', 'cyan', 'linewidth', 2);

    % draw the trajectory as estimated by the currently best particle
    trajectory = [trajectory{bestParticleIdx,:}];
    line(trajectory(1,:), trajectory(2, :), 'color', 'k', 'LineStyle','-.', 'linewidth', 2);
    tr = vecnorm(trajectory(1:2,:)-gt(1:2,timestep));
    err.tr(1:2,timestep) = [rms(tr); std(tr)];

    drawrobot(SLAM.particle(bestParticleIdx).pose, 'r', 3, 0.3, 0.3);
    xlim([-2, 12])
    ylim([-2, 12])

    hold off

    % dump to a file or show the window
    if window
        figure(1);
        drawnow;
        pause(0.1);
    else
        figure(1, "visible", "off");
        filename = sprintf('../plots/fastslam_%03d.png', timestep);
        print(filename, '-dpng');
    end
    
    % Plot error in final step
    if timestep == size(gt,2) && err.showErr
    %if timestep == size(gt,2) && isfield(err,'showErr') && err.showErr

        if is_known
            set(gcf,'color','w');
            figure(2); grid on;
            subplot(3,1,1)
            plot(1:timestep, sum(err.mean~=0),'LineWidth',2);
            xlabel('Timestep'); ylabel({'Landmarks';'observed'}); set(gca,'FontSize',16);
            subplot(3,1,2)
            errorbar(1:timestep, sqrt(sum(err.mean.^2,1))./sum(err.mean~=0),...
                     sqrt(sum(err.sig.^2,1))./sum(err.mean~=0),'r-','LineWidth',0.2);
            xlabel('Timestep'); ylabel({'MSE of landmarks','estimation'}); set(gca,'FontSize',16);
            subplot(3,1,3)
            errorbar(1:timestep, err.tr(1,:), err.tr(2,:),'b-','LineWidth',0.2);
            xlabel('Timestep'); ylabel({'MSE of trajectory','estimation (m)'}); set(gca,'FontSize',16);
            ylim([0 15]); grid on;
        else
            set(gcf,'color','w');
            figure(2); grid on;
            subplot(3,1,1)
            plot(1:timestep, err.obsv_N,'r','LineWidth',2); hold on;
            plot(1:timestep, err.store,'k','LineWidth',2);
            legend('True', 'Estimated')
            xlabel('Timestep'); ylabel({'Observed Landmark';'Numbers'}); set(gca,'FontSize',16);
            subplot(3,1,2)
            plot(1:timestep, err.mean,'r','LineWidth',2);
            xlabel('Timestep'); ylabel({'Error of landmark','estimation'}); set(gca,'FontSize',16);
            subplot(3,1,3)
            errorbar(1:timestep, err.tr(1,:), err.tr(2,:),'b-','LineWidth',0.2);
            xlabel('Timestep'); ylabel({'MSE of trajectory','estimation (m)'}); set(gca,'FontSize',16);
            ylim([0 15]); grid on;
        end
        
        % setting for printing png
        set(gcf,'Units','inches','position',[0 0 10 13]);
        fig.PaperPositionMode = 'auto';
        set(gcf,'PaperUnits','inches','PaperPosition',[0 0 10 13]);
        saveas(gcf,'fig/try_error.png');
    end
end

% totCost_store       = cell(length(x_store),1);
% costStruct_store    = cell(length(x_store),1);
% uArena_store        = cell(length(x_store),1);
% for i=1:length(x_store)
%     [totCost_store{i},costStruct_store{i},uArena_store{i}] = sim_calc_cost(simPar_store{i},x_store{i},false);
% end
close all;
figure(1);
subplot(2,1,1);
hold on;
cost_bar_data = zeros(3,length(x_store));
for i=1:length(x_store)
    cost_bar_data(:,i) = [costStruct_store{i}.distanceCost; costStruct_store{i}.velocityCost; costStruct_store{i}.collisionCost];
end
bar(cost_bar_data','stacked');
legend('distance Cost','velocity Cost','collision Cost');
Labels = {'simple NN', 'Pinciroli', 'Polynomial', 'Sinusoid'};
set(gca, 'XTick', 1:4, 'XTickLabel', Labels);
grid minor;
subplot(2,1,2);
bar(cost_bar_data(:,[1 3 4])','stacked');
legend('distance Cost','velocity Cost','collision Cost');
sLabels = {'simple NN', 'Polynomial', 'Sinusoid'};
set(gca, 'XTick', 1:4, 'XTickLabel', sLabels);
grid minor;

figure(2);
for i=1:length(x_store)
subplot(2,2,i);
x = 0:0.01:uArena_store{i}.agents{1}.cam_range;
y = uArena_store{i}.agents{1}.getAgentFunction(x);
hold on;
plot(x,y);
plot([0 max(x)],[uArena_store{i}.agents{1}.v_max uArena_store{i}.agents{1}.v_max],'--','Color','black');
plot([0 max(x)],[0 0],'--','Color','black');
sigma = uArena_store{i}.agents{1}.seperation_range + uArena_store{i}.agents{1}.collision_range;
plot([sigma sigma],[-0.5*uArena_store{i}.agents{1}.v_max 1.5*uArena_store{i}.agents{1}.v_max],'--','Color','red');
grid minor;
hold off;
axis([0 max(x) -0.5*uArena_store{i}.agents{1}.v_max 1.5*uArena_store{i}.agents{1}.v_max]);
title(strcat([ Labels{i} ' velocity response']));
xlabel('Neighbour distance [m]');
ylabel('Velocity response [m/s]');
end

figure(3);
hold all;
for i=1:length(x_store)
    x = 0:uArena_store{i}.dt:uArena_store{i}.T;
    y = uArena_store{i}.distance_cost;
    plot(x(2:end),y,'DisplayName',Labels{i});
end
legend('show');
grid minor;
ylabel('Lattice deviational energy [-]');
xlabel('Time [s]');
title('Cost build up');
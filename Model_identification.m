clc;
clear;
close all;

%% 
dim = 6;           % [a1 a2 a3 b1 b2 b3]
pop_size = 50;     % 개체 수 (population size)
Gen_max = 200;     % 세대 수

lu = [0    0    0    0    0    0;
      5    5    5    5    5    5];   % 파라미터 범위
pop_min = repmat(lu(1, :), pop_size, 1);
pop_max = repmat(lu(2, :), pop_size, 1);

% 초기 개체군
pop = pop_min + (pop_max - pop_min) .* rand(pop_size, dim);
cost = zeros(pop_size, 1);

% 시뮬레이션용 데이터 불러오기
load motor_data.mat
x1 = out.x1;

u.time = out.tout;
u.signals.dimensions = 3;
u.signals.values = out.u;

% 개체별 cost 계산
for i = 1:pop_size
    hat_A = diag([pop(i,1), pop(i,2), pop(i,3)]);
    hat_B = diag([pop(i,4), pop(i,5), pop(i,6)]);
    from_sim = sim("Model_Sim.slx");
    hat_x1 = from_sim.hat_x1;

    cost(i) = sum(sum(abs(x1-hat_x1)));  % 전체 오차합 (절대값)
end

% 초기 최적 해
[best_cost, best_idx] = min(cost);
best_pop = pop(best_idx, :);
cost_REC = zeros(1, Gen_max);

%% DE main loop
F = 0.8;   % 변이 계수
CR = 0.9;  % 교차율

for iter = 1:Gen_max
    fprintf("Generation %d / %d\n", iter, Gen_max);

    for i = 1:pop_size
        % === 변이 (Mutation) ===
        idx = randperm(pop_size, 3);
        while any(idx == i), idx = randperm(pop_size, 3); end
        a = pop(idx(1), :);
        b = pop(idx(2), :);
        c = pop(idx(3), :);

        v = a + F * ( b - c );
        v = max(min(v, lu(2,:)), lu(1,:));  % 범위 제한

        % === 교차 (Crossover) ===
        u_trial = pop(i, :);
        j_rand = randi(dim);
        for j = 1:dim
            if rand < CR || j == j_rand
                u_trial(j) = v(j);
            end
        end

        % === 선택 (Selection) ===
        hat_A = diag([u_trial(1), u_trial(2), u_trial(3)]);
        hat_B = diag([u_trial(4), u_trial(5), u_trial(6)]);
        from_sim = sim("Model_Sim.slx");
        hat_x1 = from_sim.hat_x1;
        cost_trial = sum(sum(abs(x1-hat_x1)));
        if cost_trial < cost(i)
            pop(i, :) = u_trial;
            cost(i) = cost_trial;
        end
    end

    % === 세대별 최적 해 갱신 ===
    [current_best, best_idx] = min(cost);
    if current_best < best_cost
        best_cost = current_best;
        best_pop = pop(best_idx, :);
    end

    cost_REC(iter) = best_cost;
end

%% 결과 시각화
figure;
plot(cost_REC, 'LineWidth', 2);
xlabel("Generation"); ylabel("Cost");
title("DE Optimization History"); grid on;
ax = gca; ax.FontSize = 14;

% 최적 파라미터로 다시 시뮬레이션
hat_A = diag(best_pop(1:3));
hat_B = diag(best_pop(4:6));
from_sim = sim("Model_Sim.slx");
hat_x1 = from_sim.hat_x1;
%%
figure()
hold on
title('$$x_{11}$$', 'Interpreter','latex')
plot(out.tout, x1(:,1))
plot(from_sim.tout, from_sim.hat_x1(:,1))
legend('$$x_{11}$$','$$\hat{x}_{11}$$', 'Interpreter','latex', 'Box','off')
xlabel("time [s]")
ylabel("angle [rad]")
ax = gca; ax.FontSize = 15;

figure()
hold on
title('$$x_{12}$$', 'Interpreter','latex')
plot(out.tout, x1(:,2))
plot(from_sim.tout, from_sim.hat_x1(:,2))
legend('$$x_{12}$$','$$\hat{x}_{12}$$', 'Interpreter','latex', 'Box','off')
xlabel("time [s]")
ylabel("angle [rad]")
ax = gca; ax.FontSize = 15;

figure()
hold on
title('$$x_{13}$$', 'Interpreter','latex')
plot(out.tout, x1(:,3))
plot(from_sim.tout, from_sim.hat_x1(:,3))
legend('$$x_{13}$$','$$\hat{x}_{13}$$', 'Interpreter','latex', 'Box','off')
xlabel("time [s]")
ylabel("angle [rad]")
ax = gca; ax.FontSize = 15;

best_pop


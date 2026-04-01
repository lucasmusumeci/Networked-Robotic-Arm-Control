clear; clc; close all;

%% ---------------------------------------------------------------
%  Paste your identification results here
%% ---------------------------------------------------------------
G   = 1.0;    % static velocity gain  (qdot_simu_ss / qdot_cmd)
tau = 0.004;    % time constant (s)
Trc = 0.1;    % total delay: actuator dead time + 0.5 s UDP (s)

%% ---------------------------------------------------------------
%  Stability formula (annexe)
%  Trc_max(Kc) = tau * acos(-1/(Kc*G)) / sqrt((Kc*G)^2 - 1)
%  Defined only for Kc*G > 1. Below that: stable for any delay.
%% ---------------------------------------------------------------
Kc_values   = linspace(1/G + 0.001, 10/G, 2000);
Trcmax_vals = tau .* acos(-1 ./ (Kc_values * G)) ./ sqrt((Kc_values * G).^2 - 1);

%% ---------------------------------------------------------------
%  Find Kc_max : largest Kc where Trc_max > Trc_real
%% ---------------------------------------------------------------
valid   = Trcmax_vals > Trc;
if any(valid)
    Kc_max = max(Kc_values(valid));
else
    Kc_max = NaN;
    warning('No stable Kc found for this delay. Consider reducing Trc or G.');
end

%% ---------------------------------------------------------------
%  Apply a safety margin (recommended: 20%)
%% ---------------------------------------------------------------
safety  = 0.80;
Kc_safe = safety * Kc_max;

fprintf('--- Identification results ---\n');
fprintf('  G   = %.4f\n', G);
fprintf('  tau = %.4f s\n', tau);
fprintf('  Trc = %.4f s\n', Trc);
fprintf('\n--- Kc selection ---\n');
fprintf('  Kc_max  (stability limit) = %.4f\n', Kc_max);
fprintf('  Kc_safe (80%% of Kc_max)  = %.4f  <-- use this in C++\n', Kc_safe);

%% ---------------------------------------------------------------
%  Plot: Trc_max vs Kc — read off your operating point
%% ---------------------------------------------------------------
figure;
plot(Kc_values, Trcmax_vals, 'b', 'LineWidth', 2); hold on;
yline(Trc,    'r--', 'LineWidth', 1.5, 'DisplayName', sprintf('Trc real = %.3f s', Trc));
xline(Kc_max, 'r:',  'LineWidth', 1.5, 'DisplayName', sprintf('Kc\\_max = %.4f', Kc_max));
xline(Kc_safe,'g--', 'LineWidth', 1.5, 'DisplayName', sprintf('Kc\\_safe = %.4f', Kc_safe));
scatter(Kc_safe, Trc, 80, 'g', 'filled', 'DisplayName', 'Operating point');
xlabel('Kc'); ylabel('Trc\_max (s)');
title('Stability boundary — choose Kc below the red line');
legend('Location', 'northeast'); grid on;
ylim([0, min(5, max(Trcmax_vals)*1.1)]);
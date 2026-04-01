%% Setup
clear();
clc();
close();

%% Premier ordre
tau = 0.015;
G = 15;

function Trc = Calc_Trc(Kc, tau, G)
    Trc = (tau.*acos(-1./(Kc.*G))) ./ sqrt(((Kc.*G).^2)-1);
end

Kc = (1/15):0.001:1;
Trc = Calc_Trc(Kc, tau, G);
plot(Kc, Trc);
xlabel('Kc');
ylabel('Trc');

%% Second ordre
Kp = 1;
Kd = 1;

function [Trc_1, Trc_2] = Calc_Trc_1_2(KG, Kp, Kd)
    delta = (Kd.^4./KG.^2)-4.*((Kp.*Kd.^2)./(KG.^2)-1);
    theta_1 = acos(0.5.*((-Kd.^2)./KG)+sqrt(delta));
    theta_2 = acos(0.5.*((-Kd.^2)./KG)-sqrt(delta));
    Trc_1 = (Kd.*theta_1)./(KG.*sqrt(1-theta_1.^2));
    Trc_2 = (Kd.*theta_2)./(KG.*sqrt(1-theta_2.^2));
end

KG = 0.01:0.01:1;
[Trc_1,Trc_2] = Calc_Trc_1_2(KG, Kp, Kd);
plot(KG, Trc_1, KG, Trc_2);
xlabel('KG');
ylabel('Trc');
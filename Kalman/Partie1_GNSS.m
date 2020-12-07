%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           TP Filtre de Kalman partie 1
%           Nicolas Merlinge (ONERA, TP ENSTA ROB312)
%           Version 2.0
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear % effacer toutes les variables
close all % fermer toutes les fenêtres
clc % effacer la ligne de commande
rng(123456) % imposer la graine de génération de nombres pseudo-aléatoire pour la répétabilité

% Initialisation des variables de stockage des données
tk=1;
t_sim(tk) = 0;
X_reel = [0, 0, 100, 5, 5, 0]' + sqrt(diag([10, 10, 10, 3, 3, 1].^2))*randn(6,1); % état vrai (inconnu du filtre)
d = size(X_reel,1); % dimension de l'état

%% Boucle de simulation physique de la trajectoire
Xv_sim = X_reel;
T = 10; % durée (s)
dt = 0.1; % pas de temps
Nk = (T/dt); % nombre d'instants
for tk = 1:Nk
    % simulation de l'état vrai
    t = dt*(tk-1); % temps courant
    X_reel = [eye(3), dt*eye(3); zeros(3), eye(3)]*X_reel;
    Xv_sim(:,tk) = X_reel;
end

%% Boucle de simulation physique des capteurs
Y_sim = zeros(3,Nk);
dm = 3; % dimension du vecteur de mesures
for tk = 2:Nk
    % Récupération de l'état vrai
    X_reel = Xv_sim(:,tk);
    % génération de la mesure réelle
    Y = [eye(3) zeros(3)]*X_reel + sqrt(diag([3,3,3])^2)*randn(dm,1);
    Y_sim(:,tk) = Y;
end

%% Boucle de simulation du filtre de Kalman

% Paramètres initiaux et de simulation
P_hat = diag([10, 10, 10, 3, 3, 1].^2); % matrice de covariance initiale
X_hat = [0, 0, 100, 5, 5, 0]'; % estimé initial
F = [eye(3),eye(3)*dt;zeros(3),eye(3)]; % matrice de dynamique (à compléter) 
% Qf = diag([0.2,0.2,0.2,0.01,0.01,0.001]).*4; % matrice de covariance de bruit de dynamique
Qf = diag([0.2,0.2,0.2,0.01,0.01,0.001]);
H = [eye(3),zeros(3)]; % matrice d'observation (à compléter) 
R = diag([3,3,3]).^4; % matrice de covariance du bruit de mesure

% Initialisation des variables de stockage des données
K_sim(:,:,tk) = zeros(d,dm);
inno_sim(:,tk) = zeros(dm,1);
P_sim(:,:,tk) = P_hat;
Pdiag_sim(:,:,tk) = diag(P_hat);
X_hat_sim(:,tk) = X_hat;

% Boucle de simulation
T = 10; % durée (s)
for tk = 2:Nk
    t = dt*(tk-1); % temps courant

    % Récupération de la mesure réelle
    Y = Y_sim(:,tk);
    
    % -------------------------------------------------------
    % Filtre de Kalman
    % -------------------------------------------------------
    
    % prediction (à compléter: variables X_hat et P_hat)
    X_hat = F*X_hat;
    P_hat = F*P_hat*F'+Qf;

    % validité de la mesure réelle (à compléter pour la question du trou de mesures)
%     if (t>=3 && t<=7)
        is_measurementValid = true;
%     else
%         is_measurementValid = false;
%     end
    
    % correction (à compléter: variables K, P_hat, inno, X_hat)
    if is_measurementValid
        K = P_hat*H'/(R+H*P_hat*H');
        P_hat = (eye(6)-K*H)*P_hat;
        inno = (Y-H*X_hat);
        X_hat = X_hat+K*inno;
        
        K_sim(:,:,tk) = K;
        inno_sim(:,tk) = inno;
    end
    
    % enregistrement des variables (pour plot)
    
    t_sim(tk) = t;
    
    P_sim(:,:,tk) = P_hat;
    Pdiag_sim(:,:,tk) = diag(P_hat);
    X_hat_sim(:,tk) = X_hat;
    
    % plot instantané
    figure(2)
    clf
    hold on
    plot(Xv_sim(1,1:tk), Xv_sim(2,1:tk), '-b')
    plot(X_hat_sim(1,1:tk), X_hat_sim(2,1:tk), '-r')
    scatter(Xv_sim(1,tk), Xv_sim(2,tk), 'b')
    scatter(X_hat_sim(1,tk), X_hat_sim(2,tk), '.r')
    plotcov(X_hat_sim(:,tk),3^2*P_sim(:,:,tk),'r')
    grid on
    legend('Position vraie', 'position estimée', 'Location','best')
    drawnow
end

figure(1)
labels = {'x (m)','y (m)','z (m)','Vx (m)','Vy (m)','Vz (m)'};
for i = 1:d
    subplot(3,2,i)
    hold on
    fill([t_sim flip(t_sim,2)],[X_hat_sim(i,:) - 3*sqrt(Pdiag_sim(i,:)), X_hat_sim(i,end:-1:1) + 3*sqrt(Pdiag_sim(i,end:-1:1))], [7 7 7]/8);
    plot(t_sim, Xv_sim(i,:), 'b')
    plot(t_sim, X_hat_sim(i,:), 'r')
    grid on
    xlabel('time (s)')
    ylabel(labels(i))
end
legend('uncertainty (3\sigma)', 'actual state', 'estimated state')

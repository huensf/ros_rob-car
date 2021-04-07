function [  ] = ResultSteerWheel(  )
%% Initialisation
clc
%close all
clear all
disp('Loading ...')
Pos = load('dirdyn_q.res');
Vit = load('dirdyn_qd.res');
Acc = load('dirdyn_qdd.res');
TorqueForce = load('dirdyn_Qq.res'); 
DrivenTorqueForce = load('dirdyn_Qc.res'); 

F_av_d = load('dirdyn_F_lat_av_d.res');
F_av_d = F_av_d(:,2);

F_av_g = load('dirdyn_F_lat_av_g.res');
F_av_g = F_av_g(:,2);

F_ar_d = load('dirdyn_F_lat_ar_d.res');
F_ar_d = F_ar_d(:,2);

F_ar_g = load('dirdyn_F_lat_ar_g.res');
F_ar_g = F_ar_g(:,2);

X_H = load('dirdyn_X_position_sensor_h.res');
X_H = X_H(2,2);
Y_H = load('dirdyn_Y_position_sensor_h.res');
Y_H = Y_H(2,2);
Z_H = load('dirdyn_Z_position_sensor_h.res');
Z_H = Z_H(2,2);

X_L = load('dirdyn_X_position_sensor_l.res');
X_L = X_L(2,2);
Y_L = load('dirdyn_Y_position_sensor_l.res');
Y_L = Y_L(2,2);
Z_L = load('dirdyn_Z_position_sensor_l.res');
Z_L = Z_L(2,2);

X_C = load('dirdyn_X_position_sensor_c.res');
X_C = X_C(2,2);
Y_C = load('dirdyn_Y_position_sensor_c.res');
Y_C = Y_C(2,2);
Z_C = load('dirdyn_Z_position_sensor_c.res');
Z_C = Z_C(2,2);




X_pos = Pos(:,2);
Y_pos = Pos(:,3);
Interin_q = Pos(:,50);
Interout_q = Pos(:,49);
Redressement_q = Pos(:,48);
Interin_qd = Vit(:,50);
Interout_qd = Vit(:,49);
Redressement_qd = Vit(:,48);
Volant_q = Pos(:,51);
Volant_qd = Vit(:,51);
Time = Pos(:,1);
Rack_q = Pos(:,29);
Rack_qd = Vit(:,29);
Rack_qdd = Acc(:,29);
Roue_av_g = Pos(:,17);
Roue_av_d = Pos(:,12);
Pinion_q = Pos(:,35);
Pinion_qd = Vit(:,35);
Pinion_qdd = Acc(:,35);
TorqueFeedback = TorqueForce(:,51);
DrivenTorqueFeedback = DrivenTorqueForce(:,51);
Roue_moy = (Roue_av_d+Roue_av_g)/2;
disp('Loading done !')

%% Ratio volant/roue
figure

plot(Volant_q(end/3:end)*180/3.1416,abs(Volant_q(end/3:end))./abs(Roue_moy(end/3:end)),'LineWidth',2);
grid on
ylim([15 22])
%xlim([-500 500])
xlabel("Angle de rotation du volant [deg]",'FontSize',14)
ylabel("Ratio système de direction []",'FontSize',14)
title("Ratio variable du système de direction",'FontSize',14)



%% Couple derapage
figure 
yyaxis left
%plot(Time,F_av_d,'LineWidth',2);
hold on
%plot(Time,F_av_g,'LineWidth',2);
%plot(Time,F_ar_d,'LineWidth',2);
%plot(Time,F_ar_g,'LineWidth',2);

yyaxis right
plot(Time,DrivenTorqueFeedback,'LineWidth',2);

xlim([0 15]);

%% Max Decceleration

Couple_tot = (2*TorqueForce(:,18)+2*TorqueForce(:,41));
figure 

plot(Time,F_av_d,'LineWidth',2);
hold on
plot(Time,F_av_g,'LineWidth',2);
plot(Time,F_ar_d,'LineWidth',2);
plot(Time,F_ar_g,'LineWidth',2);
%plot(-3800*ones(1,100),linspace(-5500,-1000,100),':','LineWidth',2)

%xlim([-4200 -3300])
%legend({'Roue avant droite','Roue avant gauche','Roue arrière droite','Roue arrière gauche','Couple total maximum'},'FontSize',10)
%xlabel("Couple de Freinage total [Nm]",'FontSize',14)
%ylabel("Force Longitudinale [N]",'FontSize',14)
%title("Couple de freinage maximum",'FontSize',14)
%plot(-TorqueForce(:,18),-200*sqrt(Acc(:,2).^2+Acc(:,3).^2));

%% DF

speed = linspace(0,130/3.6,100);

df = 1.1*(-speed.^2)./(2*-8.431);

figure 
plot(speed*3.6,df,'LineWidth',2)
grid on
xlabel("Vitesse [km/h]",'FontSize',14)
ylabel("Distance de freinage [m]",'FontSize',14)
title("Distance de freinage",'FontSize',14)
xlim([0 130])
%% Acceleration
sqrt(Acc(5000,2).^2+Acc(5000,3).^2)
figure
plot(TorqueForce(:,1),sqrt(Acc(:,2).^2+Acc(:,3).^2));
%plot(TorqueForce(:,1),TorqueForce(:,38));

%% SENSOR for the different angles
clc
alpha = atan((X_L-X_H)/(Z_H-Z_L)); %angle de chasse

theta = atan((Y_L-Y_H)/(Z_H-Z_L));%king pinangle

chasse_angle = tan(alpha)*((Z_H-Z_C)+0.279422);

chasse_offset = -(X_C-X_H);

chasse_tot = chasse_angle+chasse_offset;

kingpin_offset = tan(alpha)*(Z_H-Z_C)+(X_H-X_C);

scrub_radius = (Y_H-Y_C)+tan(theta)*((Z_H-Z_C)+0.279422);

disp(['Angle de chasse : ',num2str(alpha*180/pi),' degre'])
disp(['Deport de chasse au sol : ',num2str(chasse_tot*1000),' mm'])
disp('----------------------------------');
disp(['Angle Kingpin : ',num2str(theta*180/pi),' degre'])
disp(['Kingpin offset : ',num2str(kingpin_offset*1000),' mm'])
disp(['Rayon de frottement : ',num2str(scrub_radius*1000),' mm'])

%% TORQUEFEEDBACK
figure(5)
plot(Time,DrivenTorqueFeedback,'LineWidth',2)
hold on
grid on
title('Simulation Volant haptique : Couple dans le volant Robotran','FontSize',18)
ylabel('Couple [Nm]','FontSize',18)
xlabel('Temps [s]','FontSize',18)
%legend('10 km/h','30 km/h','50 km/h','90 km/h','120 km/h')
xlim([5 6])

%% POSITION VOLANT
figure(1)
plot(Time,Volant_q,'LineWidth',2)
hold on
grid on
title('Simulation Volant Haptique : Vitesse du volant ROS et Robotran','FontSize',18)
ylabel('Vitesse volant [rad/s]','FontSize',18)
xlabel('Temps [s]','FontSize',18)
legend('Robotran','ROS')
xlim([0 30])


%% TORQUEFEEDBACK VS Q Volant
figure(5)
plot(Volant_q,DrivenTorqueFeedback,'LineWidth',2)
hold on
grid on
title('Couple dans le volant en fonction de la position du volant','FontSize',14)
ylabel('Couple [Nm]','FontSize',14)
xlabel('Pos volant [Rad]','FontSize',14)


%% STABILISATION DU VEHICULE
figure(1)
plot(Time,Rack_qd,'LineWidth',2);
hold on 
grid on
title('Stabilisation de la Cr�maill�re apr�s une perturbation ','FontSize',14)
xlabel('Temps [s]','FontSize',14)
ylabel('Position Cr�maill�re [m]','FontSize',14)
xlim([8 24])
%legend({'D�port du pivot : 10.0 mm','D�port du pivot : 20.0 mm'},'FontSize',12)
%% Ratio Pignion Rack
figure
plot(Pinion_q(2:end),(Pinion_q(2:end))./(Rack_q(2:end)) ,'LineWidth',2)
hold on
title('Ratio entre le pignon et la cr�maill�re','FontSize',16)
xlabel('Angle du pignon [deg]','FontSize',14)
ylabel('Angle du pignon/Position de la cr�maill�re','FontSize',14)
xlim([-8 8])
grid on

%% PINON RACKQD
figure
plot(Pinion_q,Rack_qd,'LineWidth',2)
hold on
hold on
grid on
title('Vitesse de la cr�maill�re en fonction de la rotation du pinion','FontSize',24)
ylabel('Vitesse de la cr�maill�re [m/s]','FontSize',14)
xlabel('Angle du pinion [rad]','FontSize',14)

%% plot Volant Vitesse pignon
figure
plot(Volant_q(2:end),Interin_qd(2:end),'LineWidth',2)
hold on
grid on
title('Vitesse angulaire de la sortie du joint en fonction de la position du volant','FontSize',11)
xlabel('Rotation du volant [rad]','FontSize',11)
ylabel('Vitesse angulaire de la sortie du joint [rad]','FontSize',11)
xlim([-8 8])

%% plot Volant pinion
figure
plot(Time(2:end),Volant_q(2:end)./Pinion_q(2:end),'LineWidth',2)
grid on
title('Ratio avec le double joint de cardan','FontSize',16)
xlabel('Rotation du volant [rad]','FontSize',14)
ylabel('Rotation du pinion [rad]','FontSize',14)

%% plot POSITION VOITURE
figure(1)
%yyaxis left
plot(X_pos,Y_pos,'LineWidth',2)
%ylim([-5 5]),
ylabel('Position lat�rale du v�hicule [m]','FontSize',16)
hold on
%yyaxis right
%plot(Time*25,Volant_q*180/pi,'LineWidth',2)
%ylabel('Rotation du volant [deg]','FontSize',16)
grid on
title('Position de la voiture et du volant en double changement de bande, V = 36 km/h','FontSize',16)
xlabel('Position longitudinale du v�hicule [m]','FontSize',16)
%xlim([0 333])

%% Rack roue Ratio
figure 
plot(Time,Rack_q,'LineWidth',2)
hold on
plot(Time,Roue_av_g,'LineWidth',2)
plot(Time,Roue_av_d,'LineWidth',2)
grid on
xlim([0 10])
title('Rack and wheel position','FontSize',16)
xlabel('Temps [s]','FontSize',14)
ylabel('Position','FontSize',14)
legend({'Cremailliere','Roue gauche','Roue droite'},'FontSize',12)
sum(Roue_av_d(3000)/Rack_q(3000))
disp(['Ratio']);
disp(['Roue droite : ',num2str(-1/(sum(Rack_q./Roue_av_d)/length(Time)))]);
disp(['Roue gauche : ',num2str(-1/(sum(Rack_q./Roue_av_g)/length(Time)))]);
disp(['Tot : ',num2str(((-1/(sum(Rack_q./Roue_av_d)/length(Time)))+(-1/(sum(Rack_q./Roue_av_g)/length(Time))))/2)])
%% Curve Fitting False

Pignon_rotation = linspace(-0.7,0.7,59);
i = [14 14 14 14 14 14 14.04 14.1 14.25 14.5 14.8 15.25 15.73 16.2 16.75 17.4 18.25 18.8 19.27 19.76 20.23 20.52 20.9 21.25 21.5 21.75 21.83 21.87 21.9 22 21.9 21.87 21.83 21.75 21.5 21.25 20.9 20.52 20.23 19.76 19.27 18.8 18.25 17.4 16.75 16.2 15.73 15.25 14.8 14.5 14.25 14.1 14.04 14 14 14 14 14 14];
%i = ones(1,length(i))./
i = (i+2*ones(1,length(i)));

p = polyfit(Pignon_rotation,i,10);

Pignon_rotation_poly = linspace(-1,1,100);
i_poly = polyval(p,Pignon_rotation_poly);

figure(1)
plot(Pignon_rotation,i,'LineWidth',2);
hold on
plot(Pignon_rotation_poly,i_poly,'LineWidth',2);
legend({'Computation','Point','Approximation'},'FontSize',12)
xlim([-0.7 0.7]);

%% Curve Fitting GOOD
%close all
%Rack_travel = linspace(-0.7,0.7,59);
Pignon_rotation = linspace(0,pi,35);
%i = [14 14 14 14 14 14 14.04 14.1 14.25 14.5 14.8 15.25 15.73 16.2 16.75 17.4 18.25 18.8 19.27 19.76 20.23 20.52 20.9 21.25 21.5 21.75 21.83 21.87 21.9 22 21.9 21.87 21.83 21.75 21.5 21.25 20.9 20.52 20.23 19.76 19.27 18.8 18.25 17.4 16.75 16.2 15.73 15.25 14.8 14.5 14.25 14.1 14.04 14 14 14 14 14 14];
%i = (i+2*ones(1,length(i)));
i = 11*[18 18 18 18 17.8 17.8 17.7 17.6 17.6 17.5 17.5 17.4 17.2 17 16.8 16.7 16.6 16.5 16.3 16 15.8 15.6 15.5 15.4 15.3 15 14.9 14.8 14.7 14.7 14.6 14.5 14.5 14.5 14.5];
%i = [fliplr(i) i];

p = polyfit(Pignon_rotation,i,4)
Pignon_rotation_poly = linspace(-8,8,100);
i_poly = polyval(p,Pignon_rotation_poly);

real_i_poly_func_droite =@(x) p(1)*(x).^4 + p(2)*(x).^3 + p(3)*(x).^2 + p(4)*(x) + p(5);
real_i_poly_func_gauche =@(x) p(1)*(-x).^4 + p(2)*(-x).^3 + p(3)*(-x).^2 + p(4)*(-x) + p(5);

real_i_poly_droite = real_i_poly_func_droite(Pignon_rotation_poly);
real_i_poly_gauche = real_i_poly_func_gauche(Pignon_rotation_poly);

p(1)
p(2)
p(3)
p(4)
p(5)



%figure(1)
%plot(Pignon_rotation,i,'LineWidth',2);
hold on
%plot(Pignon_rotation_poly,i_poly,'LineWidth',2);
plot(Pignon_rotation_poly,real_i_poly_droite,'LineWidth',2);
plot(Pignon_rotation_poly,real_i_poly_gauche,'LineWidth',2);
grid on
legend({'Point','Approximation','For Rob'},'FontSize',12)
title('Ratio entre la cr�maill�re et le pignon','FontSize',14)
xlabel('Angle de rotation du pignon [deg]','FontSize',14)
ylabel('Ratio','FontSize',14)
xlim([-8 8]);


%% homocinetique double joint de cardan
figure
plot(Volant_q(2:end),Interin_qd(2:end),'LineWidth',2.5)
hold on
plot(Interout_q(2:end),Interout_qd(2:end),'LineWidth',2.5)
plot(Volant_q(2:end),Pinion_qd(2:end),'LineWidth',2.5)
grid on
xlim([-8 8])
xlabel('Angle d''entr�e [rad]','FontSize',16)
ylabel('Vitesse angulaire de sortie [rad/s]','FontSize',16)
title('Relation entre l''angle d''entr�e et la vitesse angulaire de sortie des joints de cardan','FontSize',18)
legend({'Joint de cardan 1','Joint de cardan 2','Joint de cardan 1+2'},'FontSize',14)
%% MAX Steer

a_b =@(x) 9.3750e-04*x.^2-0.2313*x+17.2188;

speed= linspace(50/3.6,130/3.6,100);
angle = a_b(speed);
figure
plot(speed*3.6,angle)
end


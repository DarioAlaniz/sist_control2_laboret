clc;clear all;%close all;
p1=3;p2=-1;K=5;
ts=3;
T=1;t=0:T:1000*T;u=0:T:1000*T;
polos = [p1 p2];ceros=[];
%________________________________
Gs=zpk(ceros,polos,K)
%F = feedback(Gs,1)
%disp('Diseñando controlador...')
Desing = load('ControlSystem.mat');
C = Desing.ControlSystemDesignerSession.DesignerData.Architecture.TunedBlocks(2).Value; %Controlador
disp('Funcion de trasnferencia a lazo cerrado con Controlador')
F = feedback(C*Gs,1)
%________________________________
Kc= C.K;     % ganancia
a= - C.Z{1}; % cero del controlador con signo invertido
M=Kc         % ganancia rele
% T=0.1       % histeresis
% T = Kc*K
T = (Kc*K)/100
% T = (Kc*K)/25
% T = (Kc*K)/10
lineal=0;    % simula control lineal
sim('bang_bang_hist_DI_PD')
%________________________________
disp('Error')
% s=tf('s')
errorSistemaLineal   = 1/(1+dcgain(Gs*C))
errorSistemaNoLineal = abs(min(yout(:,1)))%yout(end,1)     
% error_rampa =1/ (1/T *(evalfr(minreal(Gs * C*s),0))) %error ante un entrada rampa
%________________________________
figure(1)
subplot(2,2,1)
plot(tout,yout(:,1));grid on;hold on;       %error
title('Error');xlabel('Tiempo.[s]')     
subplot(2,2,2)
plot(-flip(yout(:,1)),flip(yout(:,3)));grid on;hold on;  %plano de fases: eje x error, eje y derivada del error
title('Plano de fases rotado');xlabel('error');ylabel('derivada error')  
subplot(2,2,3:4)
plot(tout,yout(:,2));grid on;hold on;       %señal de control
title('Accion de control');xlabel('Tiempo.[s]')
figure(2)
plot(tout,yout(:,4));grid on;hold on;       %respuesta al impulso
title('Salida');xlabel('Tiempo.[s]');
% figure(3)
% % subplot(2,1,1)
% % plot(flip(yout(:,1)),flip(yout(:,3)));grid on;hold on;
% % subplot(2,1,2)
% plot(tout,yout(:,2));grid on;hold on;xlim([0 2]);
% plot(-flip(yout(:,1)),flip(yout(:,3)));grid on;hold on;  %plano de fases: eje x error, eje y derivada del error
% title('Plano de fases rotado');xlabel('error');ylabel('derivada error')
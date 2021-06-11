clc;clear all;%close all;
p1=3;p2=-1;K=5;
ts=3;
T=1;t=0:T:1000*T;u=0:T:1000*T;
polos = [p1 p2];ceros=[];
%________________________________
Gs=zpk(ceros,polos,K)
%disp('Diseñando controlador...')
Desing = load('ControlSystemPID.mat');
C = Desing.ControlSystemDesignerSession.DesignerData.Architecture.TunedBlocks(2).Value; %Controlador
disp('Funcion de trasnferencia a lazo cerrado con Controlador')
F = feedback(C*Gs,1)
%________________________________
ec=poly([C.Z{1}(1) C.Z{1}(2)]);
Kd = C.K     
Kp = C.K*ec(2)
Ki = C.K*ec(3)
%________________________________
disp('Error')
s=tf('s')
error       = 1/(1+dcgain(Gs*C))   
error_rampa = 1/ ((evalfr(minreal(Gs * C*s),0))) %error ante un entrada rampa
%________________________________
sim('../actividad_2/PID_Continuo.slx')
figure(3)
subplot(2,1,1)
plot(tout,yout(:,5),tout,yout(:,6),tout,yout(:,4));grid on;
xlabel('Tiempo.[seg]');title('Acciones de control');
legend('accion integral','accion proporcional','accion derivativa');
subplot(2,1,2)
plot(tout,yout(:,3));grid on;
title('Error');xlabel('Tiempo.[seg]');
figure(4)
subplot(2,1,1)
plot(tout,yout(:,1));grid on;
title('Salida');xlabel('Tiempo.[seg]');
subplot(2,1,2)
pzmap(F);
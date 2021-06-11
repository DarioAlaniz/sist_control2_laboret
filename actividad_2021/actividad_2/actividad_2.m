clc;clear all;close all;
p1=-3;p2=-2;c1=-10;K=10;
Mp=15/100;ts=3;T=0.09;t=0:T:1000*T;u=0:T:1000*T;z=tf('z');
polos = [p1 p2];ceros=[c1];
Gs=zpk(ceros,polos,K)
disp('Funsion de transferencia sin controlador')
Gd = c2d(Gs,T,'zoh')
%F = feedback(Gd,1)
disp('Especificaciones:')
psita = sqrt(log(Mp)^2/(pi^2+log(Mp)^2))
ts
w0 = 4/(psita*ts); %frecuencia no amortiguada
disp('Frecuencia amortiguada')
wd = w0 * sqrt(1-psita^2) %frecuencia amortiguada
td = (2*pi)/wd;
disp('Muestras por ciclo')
m  = (td/T)
r  = exp(-psita*w0*T) ; omega = wd * T;
[real,imag] = pol2cart(omega,r);
disp('Polos deseados');
pd1 = real + i*imag
pd2 = real - i*imag
%disp('Diseñando controlador...')
Controlador = load('Diseño_controlador.mat');
Desing1 = Controlador.ControlSystemDesignerSession.DesignerData.Designs(1).Data; %PI
Desing2 = Controlador.ControlSystemDesignerSession.DesignerData.Designs(2).Data; %PID
C1= Desing1.C;
C2= Desing2.C;
%disp('Funcion de trasnferencia a lazo cerrado con Controlador')
F1 = feedback(C1*Gd,1);
F2 = feedback(C2*Gd,1);

error_calculado_con_PI = 1/(1+dcgain(Gd*C1));
error_rampa_PI =1/ (1/T *(evalfr(minreal(Gd * C1*(z-1)),1)));

error_calculado_con_PID = 1/(1+dcgain(Gd*C2));
error_rampa_PID =1/ (1/T *(evalfr(minreal(Gd * C2*(z-1)),1)));

% %parametros para compensador PI
% Kd=0;
% Ki= C1.K /(1+( C1.Z{1}(1)/(1- C1.Z{1}(1))));
% Kp= Ki / ((1/ C1.Z{1}(1))-1);
% para compensadro PID
ec=poly([C2.Z{1}(1) C2.Z{1}(2)]);
Kd = C2.K*ec(3)
Kp = C2.K*(-ec(2)-2*ec(3))
Ki = C2.K*(ec(3)-(-ec(2))+1)

figure(1)
subplot(2,2,1:2);
step(F1);hold on;step(F2,7)
subplot(2,2,3);
pzmap(F1);
subplot(2,2,4);
pzmap(F2);

figure(2)
plot(tout,yout(:,5),tout,yout(:,6),tout,yout(:,4));legend('accion integral','accion proporcional','accion derivativa');
figure(2);hold on;
plot(tout,yout(:,5),tout,yout(:,6));legend('accion integral','accion proporcional');
% figure(3)
% lsim(F1,u,t)


%ver de implementar un PID completo o un PI + PD 
%%
clc;clear all;
syms c1 c2 c b cc1 cc2 Kd Ki Kp K c_aux2 b_aux2
% ec1 = c1 == (b + sqrt(b^2-4*c))/2
% ec2 = c2 == (b - sqrt(b^2-4*c))/2
% b_aux = solve(ec1,b) %b
% c_aux = solve(ec2,c) %c
% 
% ec1 = c == subs(c_aux,'b',b_aux)
% ec2 = b == subs(b_aux,'c',c_aux)
% 
% c = solve(ec1,c)
% b = solve(ec2,b)

% K= Kp+Kd+Ki
% b_aux = (Kp + 2*Kd)/K
% c_aux = Kd/K
% 


Kd = K * c
Kp = b*K - 2*Kd;Kp=simplify(Kp) 
Ki = K - Kd - Kp;Ki=simplify(Ki) 


%  c1 = C1.Z{1}(1)
%  c2 = C1.Z{1}(2)
 
 

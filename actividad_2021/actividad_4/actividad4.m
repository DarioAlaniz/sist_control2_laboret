%Linealizacion del sistema
clc;clear all;
syms fi fip fipp a b c u T x1 x2 x1p x2p delta e
x1 = e; %e=fi-delta
x2 = fip;
x1p = fip;
x2p = u*c -b*c*x2 + a*(sin(delta) - sin(x1+delta));

A = [[subs(subs(diff(x1p,x1),x1,0),x2,0) subs(subs(diff(x1p,x2),x1,0),x2,0)];
     [subs(subs(diff(x2p,x1),x1,0),x2,0) subs(subs(diff(x2p,x2),x1,0),x2,0)]];

B = [subs(subs(diff(x1p,u),x1,0),x2,0);
     subs(subs(diff(x2p,u),x1,0),x2,0)];

C = [subs(subs(diff(x1,x1),x1,0),x2,0) subs(subs(diff(x2,x1),x1,0),x2,0)];

D = [0];

%% Simulacion del modelo lineal
clc; clear all;
m=3;b=0.1;l=1;G=10;delta=135;p=[-4 -4 -4];
a = G/l;c=1/(m*l^2);
if 0
    disp('Modelo calculado')
    Aobt = [0, 1;
            -a*cos(degtorad(delta)), -b*c];
    Bobt = [0;
            c];
    autoValObte=eig(Aobt)
end
[A,B,C,D]=linmod('pendulo_mod_tarea',degtorad(delta));
sys = ss(A,B,C,D);
if 0
   disp('Modelo obtenido')
   A
   B 
end
eig(A) %presenta un polo en el semi plano derecho por lo que es inestable
% pzmap(sys);
if length(A) == rank(ctrb(A,B))
    disp('Es controlable')
end
Aa=[A zeros(2,1);
    C          0];
Ba=[B; 
    0];
sysa=ss(Aa,Ba,[C 0;0 0 1],D);
eig(Aa) %ahora es de tipo 1, sigue siendo inestable por el polo en +2.6425
% pzmap(sysa);
if length(Aa) == rank(ctrb(Aa,Ba))
    disp('Es controlable el sistema ampliado')
end
K=acker(Aa,Ba,p);
k1=K(1);k2=K(2);k3=K(3);
eig(Aa-Ba*K)
tscalc=7.5/-p(1)

sim('pendulo_PID_tarea') 
figure(1)
plot(tout,yout);hold on;
grid on, title('Salida'),xlabel('Tiempo.[s]');
figure(2) 
plot(yout,velocidad);hold on; %plano de fase
grid on, title('Plano de fases'),xlabel('angulo'),ylabel('velocida angular');
figure(3) 
plot(tout,torque);hold on; % torque total
grid on, title('Torque'),xlabel('Tiempo.[s]');
figure(4) 
plot(tout,-accint);hold on;plot(tout,-accderi);hold on;plot(tout,-accprop);hold on; % acción integral,derivativa,proporcional
grid on, title('Acciones de control'),xlabel('Tiempo.[s]');
% legend('acc. integral','acc. derivativa','acc. proporcional');
disp('Resultados obtenidos: ')
ymax=max(yout)      % máximo valor de salida
S=(ymax-delta)/delta*100 % sobrepaso en %
erel=(delta-yout)/delta; %error relativo
efinal=erel(end)    % error final, debe ser cero
ind=find(abs(erel)>.02); % índice elementos con error relativo absoluto menor a 2%
tss=tout(ind(end))  % tiempo de establecimiento (ultimo valor del vector)
yte=yout(ind(end))  % salida al tiempo ts
uf=torque(end)      % torque final
Intf=-accint(end)   % acción integral final
%{
for i=1:2
    m=3;
    m=((-1)^(i)*0.1+1)*m
    sim('pendulo_PID_tarea')
    figure(1)
    plot(tout,yout);hold on;
    grid on, title('Salida'),xlabel('Tiempo.[s]');
    figure(2) 
    plot(yout,velocidad);hold on; %plano de fase
    grid on, title('Plano de fases'),xlabel('angulo'),ylabel('velocida angular');
    figure(3) 
    plot(tout,torque);hold on; % torque total
    grid on, title('Torque'),xlabel('Tiempo.[s]');
    figure(4) 
    plot(tout,-accint);hold on;plot(tout,-accderi);hold on;plot(tout,-accprop);hold on; % acción integral,derivativa,proporcional % acción integral
    grid on
end
figure(1), legend('m=3','m=2.7','m=3.3');
figure(4), legend('m=3','m=3','m=3','m=2.7','m=2.7','m=2.7','m=3.3','m=3.3','m=3.3');
%}


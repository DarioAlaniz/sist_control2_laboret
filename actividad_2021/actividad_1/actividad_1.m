%---- p1=-3,p2=-2,c1=-10,K=10,tss=3,Mp=15;
%parametros---------------
clc;clear all;%close all;
K=10;polos = [-3 -2];ceros=[-10];T=10*0.09;

G=zpk(ceros,polos,K)
Gz=c2d(G,T,'zoh')

Kp=dcgain(Gz)
error_calculado=1/(1+Kp)

Gz_F= feedback(0.12*Gz,1)
error_obtenido= 1 - dcgain(Gz_F)

z=tf('z');
Kv = 1/T *(evalfr(minreal(Gz * (z-1)),1))
error_rampa=1/Kv

t=0:T:1000*T;
u=0:T:1000*T;
figure(1)
rlocus(G)
% step(Gz_F);hold on;
%lsim(Gz_F,u,t)
% pzmap(G)
figure(2)
pzmap(Gz_F)
%rlocus(Gz)
%pzmap(Gz);hold on;%legend('T=0.09','T=0.9')
% subplot(2,1,1)
% step(G,4)
% subplot(2,1,2)
%step(Gz,4)
%step(Gz_F)
%Matlab Script to project control
fs = 40; %Hz
Ts = 1/fs;

%Drone ============================================
s = tf('s');
z = tf('z',Ts);
G = 1/(s*(s+1));


Gd = c2d(G,Ts,'zoh');

poleGd = pole(Gd);
Gd = zpk(minreal(Gd));
[zg,pg,kg] = zpkdata(Gd);

%Projeto do Controlador ==========================
%Cd = K*(z-c1)*(z-c2)/(z*(z-1)) %PID
%Cc = k*(s + zc)/(s + pc)       %Pole alocation

%Maximum peak: Mp = 3%
%Settling time: ts= 8s

Mp = 3;%

xi = -log(Mp/100)/sqrt(pi^2 + log(Mp/100)^2); %garante Sobressinal nulo
%ts = 4/xi*wn
ts = 10;
wn = 4/(xi*ts);

s1 = -xi*wn + 1i*wn*sqrt(1-xi^2);
s2 = -xi*wn - 1i*wn*sqrt(1-xi^2);

b = 2/Ts;

A = [1  -1       0
     b  -2*xi*wn 0
     0   wn*wn  -b];
v = [2*xi*wn-b
     wn*wn
     0];
 
aux = A\v; 

pc = aux(1);
p = aux(2);
k = aux(3);

zc = pg{1}(2);

Cc = k*(s + zc)/(s + pc);

Cd = c2d(Cc,Ts,'tustin');

FTMF_continuo = feedback(G*Cc,1);
FTMF_digital = feedback(Gd*Cd,1);

step(FTMF_continuo,FTMF_digital);

[zcd,pcd,kcd] = zpkdata(Cd);

%%
disp('Controller in python')
controllaw = ['un = ',num2str(-pcd{1}), '*un_1 +', num2str(k), '*en +',num2str(k*(zcd{1})) ,'*en_1'];
disp(controllaw)

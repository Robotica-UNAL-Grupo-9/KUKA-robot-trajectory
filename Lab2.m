%% 6
clc, clear ,close all


L1=0.340; L3=0.400; L5=0.400; L7=0.161;

% theta_i d_i a_(i-1) alpha_(i-1) [rad]

DH_mod=[0 L1 0 0 ;
        0  0 0 -pi/2;
        0 L3 0  pi/2;
        0  0 0 -pi/2;
        0 L5 0  pi/2;
        0  0 0 -pi/2;
        0 L7 0  pi/2];

      
offset=[0 0 0 0 0 0 0];

qlim=[ -170 170;       -120 120;
       -170 170;       -120 120;
       -170 170;       -120 120;
       -175 1750];
     
for k=1:length(offset)
  
  L(k)=Link(DH_mod(k,:),'offset',offset(k),'qlim',qlim(k,:),'modified');
end     
  
ws=[-5 2 -4 4 -2 5];

plot_options = {'workspace',ws,'scale',.4,'view',[125 25],'jaxes','basewidth',10};
RKuka = SerialLink(L,'name','Kuka','plotopt',plot_options)

%Determinar la posición del último sistema coordenado dado q:
% q3=0
q=[pi/3 pi/6 0 pi/4 -pi/3 3*pi/4 pi/8];

MTH=RKuka.fkine(q)

[R,pos]=tr2rt(MTH);

z_EF=R*[0;0;L7];
angles=tr2rpy(MTH,'deg','zyx');  % Phi, Theta Psi z,x,y

OC=pos-z_EF;
x=OC(1); y=OC(2); z=OC(3);
r=sqrt(x^2+y^2);

A=sqrt((z-L1)^2+r^2);

c4=(A^2-L3^2-L5^2)/(2*L3*L5);
s4=sqrt(1-c4^2);
s_beta=s4*L5/A;
beta=atan2(s_beta,sqrt(1-s_beta^2));
gamma=atan2(r,z-L1);

q3=0;
q1=atan2(y,x);
q4=atan2(s4,c4);

q2=-beta+gamma;


q_calc=[q1,q2,q3,q4,0,0,0];


Twrist=RKuka.fkine(q_calc);
[Rwrist,Pwrist]=tr2rt(Twrist);

R36=inv(Rwrist)*R;

angles=tr2eul(R36,'flip','deg')  % Phi, Theta Psi z,x,y
q_567=tr2eul(R36);

c6=R36(3,3);


q5=atan2(R36(2,3),R36(1,3));
q6=atan2(sqrt(1-c6^2),c6);
q7=atan2(R36(3,2),-R36(3,1));

%q_567=[q5 q6 q7] ;

q_calc=[q1,q2,q3,q4,q_567];

disp("q:  " )
disp(q*180/pi)

disp("q_calc")
disp(q_calc*180/pi)



Tcalc=RKuka.fkine(q_calc);

e=round(Tcalc -MTH,10)

var=RKuka.fkine([q(1:4), 0 0 0]);


%% Punto 3.7
clc

L1=0.340; L3=0.400; L5=0.400; L7=0.161;

% x y z rollo pitch yaw
coordinates=[ 0.4 0.6 0.5 30 20 45;
              -0.35 0.4 0.8 40 60 10;
              0.3 0.2 -0.1 -30 180 4;
              0.4 0.5 0 30 45  10];
            
rta=zeros(size(coordinates,1),7);

for k=1:size(coordinates,1)
  E=coordinates(k,:);
  
  MTH=transl(E(1:3))*rpy2tr(E(4:6),'deg');
  
  [q_calc,e]=inverse_k(MTH, RKuka);
  rta(k,:)=q_calc;  
  disp ("k:"+k +"  error:" + any(e, 'all'))
  
  if any(e, 'all')
    disp('toteo en' +k+ ':(')
    disp(e)
  end
   
end
rta

%%
close all

t=linspace(0,1,1)';

n=4;
RKuka.plot([q(1:n), zeros(1,7-n)].*t)
hold on
RKuka.plot([q_calc(1:n), zeros(1,7-n)].*t)


%% Cinematica Inversa Toolbox RCV
clear variables
syms q1 q2 q3 q4 q5 q6 q7 MF L_1 L_2 L_3 real
MF=0;
L_1=34;
L_2=40;
L_3=40;
L4(1) = Link('revolute'   ,'alpha',      0,  'a',  0,     'd',    L_1 , 'offset',     0, 'qlim',  [-170*pi/180 170*pi/180],   'modified');
L4(2) = Link('revolute'  ,'alpha',   pi/2,  'a',  0, 'd',    0 , 'offset',    0, 'qlim',       [-120*pi/180 120*pi/180],   'modified');
L4(3) = Link('revolute'   ,'alpha',  -pi/2,  'a',  0,     'd',    L_2 , 'offset',     0, 'qlim',  [-170*pi/180 170*pi/180],   'modified');
L4(4) = Link('revolute'   ,'alpha',  pi/2,  'a',  0,     'd',    0 , 'offset',     0, 'qlim',  [-120*pi/180 120*pi/180],   'modified');
L4(5) = Link('revolute'   ,'alpha',  -pi/2,  'a',  0,     'd',    L_3 , 'offset',     0, 'qlim',  [-170*pi/180 170*pi/180],   'modified');
L4(6) = Link('revolute'   ,'alpha',  pi/2,  'a',  0,     'd',    0 , 'offset',     0, 'qlim',  [-120*pi/180 120*pi/180],   'modified');
L4(7) = Link('revolute'   ,'alpha',  -pi/2,  'a',  0,     'd',    MF , 'offset',     0, 'qlim',  [-175*pi/180 175*pi/180],   'modified');

ws=[-10 50 -30 30 -2 70];

plot_options = {'workspace',ws,'scale',.4,'view',[125 25],'basewidth',10};
RKuka = SerialLink(L4,'name','Kuka','plotopt',plot_options);

T0tcp=RKuka.fkine([0,0,0,0,0,0,0]); %%Posicion de home
q=[0 pi/4 0 pi/2 0 pi/2 0]; %Punto de prueba
MTH=RKuka.fkine(q); %MTH punto de prueba

qikcon=(RKuka.ikcon(MTH));
MTHikcon=RKuka.fkine(qikcon);



%% functions

function [q_calc,e]=inverse_k(MTH, RKuka)
  
L1=0.340; L3=0.400; L5=0.400; L7=0.161;

[R,pos]=tr2rt(MTH); % matriz de rotación y vector de posición

z_EF=R*[0;0;L7];

OC=pos-z_EF;            % centro de muñeca
x=OC(1); y=OC(2); z=OC(3);

r=sqrt(x^2+y^2);        % en el plano XY

% teorema del cos
A=sqrt((z-L1)^2+r^2);   

c4=(A^2-L3^2-L5^2)/(2*L3*L5);
s4=sqrt(1-c4^2);

% teorema del sen
s_beta=s4*L5/A;
beta=atan2(s_beta,sqrt(1-s_beta^2));

gamma=atan2(r,z-L1);

% posición
q3=0;         % reducción de grados de libertad 
q1=atan2(y,x);
q4=atan2(s4,c4);

q2=-beta+gamma;


q_calc=[q1,q2,q3,q4,0,0,0];

% rotación
Twrist=RKuka.fkine(q_calc);      

[Rwrist,Pwrist]=tr2rt(Twrist);

R36=inv(Rwrist)*R;


q_567=tr2eul(R36);

c6=R36(3,3);


q5=atan2(R36(2,3),R36(1,3));
q6=atan2(sqrt(1-c6^2),c6);
q7=atan2(R36(3,2),-R36(3,1));

%q_567=[q5 q6 q7] ;

q_calc=[q1,q2,q3,q4,q_567];

Tcalc=RKuka.fkine(q_calc);

e=round(Tcalc -MTH,10);

end

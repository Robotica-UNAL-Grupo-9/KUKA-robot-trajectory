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
       -175 175];
     
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
clc , close all

L1=0.340; L3=0.400; L5=0.400; L7=0.161;

% x y z rollo pitch yaw
coordinates=[ 0.4 0.6 0.5 30 20 45;
              -0.35 0.4 0.8 40 60 10;
              0.3 0.2 -0.1 -30 180 4;
              0.4 0.5 0 30 45  10];
            
N=50;
t=linspace(0,1,N)';
            
rta=zeros(size(coordinates,1),7);
trayect=zeros(N*size(coordinates,1),7);
q_ant=[0 0 0 0 0 0 0];

for k=1:size(coordinates,1)
  E=coordinates(k,:);
  
  MTH=transl(E(1:3))*rpy2tr(E(4:6),'deg');
  
  [q_calc,e,Tcalc]=inverse_k(MTH, RKuka);
  rta(k,:)=q_calc;
  
  trayect((1:N)+(k-1)*N,:)=q_ant+(q_calc-q_ant).*t;
  
  %disp ("k:"+k +"  error:" + any(e, 'all'))
  
  if any(e, 'all')
    disp('toteo en' +k+ ':(')
    disp(e)
  end
   q_ant=q_calc;
  
  comparacion=table(Tcalc,MTH);
  
   disp("Pose "+k)
   disp( E)
   disp(comparacion)
  %input("hola")
end
rta*180/pi
%RKuka.plot(trayect)


%%
close all

t=linspace(0,1,1)';

n=4;
RKuka.plot([q(1:n), zeros(1,7-n)].*t)
hold on
RKuka.plot([q_calc(1:n), zeros(1,7-n)].*t)



%% Cinematica Inversa Toolbox RCV
clear variables
clc
syms q1 q2 q3 q4 q5 q6 q7 MF L_1 L_2 L_3 real
MF=0.161;
L_1=0.34;
L_2=0.40;
L_3=0.40;
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


q=[0 -pi/3 0 pi/3 0 pi/3 0]; %Punto de prueba
MTH=RKuka.fkine(q); %MTH punto de prueba

qikine=(RKuka.ikine(MTH)); %Cinematica inversa funcion ikine
MTHikine=RKuka.fkine(qikine);%Verificacion MTH a partir de solucion ikine

qikunc=(RKuka.ikunc(MTH));%Cinematica inversa funcion ikunc
MTHikunc=RKuka.fkine(qikunc);%Verificacion MTH a partir de solucion ikunc

qikcon=(RKuka.ikcon(MTH));%Cinematica inversa funcion okcon
MTHikcon=RKuka.fkine(qikcon);%Verificacion MTH a partir de solucion ikcon

%% Cinematica Inversa Toolbox RST
clear variables
clc
MF=0.161;
dhparams = [0   	0	0.340   	0;
            0	pi/2       0       0
            0	-pi/2	0.400	0;
            0   	pi/2	0	0;
            0       -pi/2	0.400   	0;
            0       pi/2       0       0;
            0       -pi/2       MF       0];
RkukaRST = rigidBodyTree;     
body1 = rigidBody('body1');
jnt1 = rigidBodyJoint('jnt1','revolute');

setFixedTransform(jnt1,dhparams(1,:),'mdh');
body1.Joint = jnt1;

addBody(RkukaRST,body1,'base')
body2 = rigidBody('body2');
jnt2 = rigidBodyJoint('jnt2','revolute');
body3 = rigidBody('body3');
jnt3 = rigidBodyJoint('jnt3','revolute');
body4 = rigidBody('body4');
jnt4 = rigidBodyJoint('jnt4','revolute');
body5 = rigidBody('body5');
jnt5 = rigidBodyJoint('jnt5','revolute');
body6 = rigidBody('body6');
jnt6 = rigidBodyJoint('jnt6','revolute');
body7 = rigidBody('body7');
jnt7 = rigidBodyJoint('jnt7','revolute');

setFixedTransform(jnt2,dhparams(2,:),'mdh');
setFixedTransform(jnt3,dhparams(3,:),'mdh');
setFixedTransform(jnt4,dhparams(4,:),'mdh');
setFixedTransform(jnt5,dhparams(5,:),'mdh');
setFixedTransform(jnt6,dhparams(6,:),'mdh');
setFixedTransform(jnt7,dhparams(7,:),'mdh');
body2.Joint = jnt2;
body3.Joint = jnt3;
body4.Joint = jnt4;
body5.Joint = jnt5;
body6.Joint = jnt6;
body7.Joint = jnt7;
addBody(RkukaRST,body2,'body1')
addBody(RkukaRST,body3,'body2')
addBody(RkukaRST,body4,'body3')
addBody(RkukaRST,body5,'body4')
addBody(RkukaRST,body6,'body5')
addBody(RkukaRST,body7,'body6')


q=[0 -pi/3 0 pi/3 0 pi/3 0];%Cnfiguración punto de prueba 
qRST = struct('JointName',{'jnt1','jnt2','jnt3','jnt4','jnt5','jnt6','jnt7'},'JointPosition',{q(1),q(2),q(3),q(4),q(5),q(6),q(7)});


MTH_RST = getTransform(RkukaRST,qRST,'body7')%MTH a partir del punto de prueba

ik = inverseKinematics('RigidBodyTree',RkukaRST);%Declaracion del solucionador de la cinematica inversa
weights = [0.25 0.25 0.25 1 1 1];%Tolerancias deseadas para la solucion
initialguess = RkukaRST.homeConfiguration;%Aproximacion inicial a la solucion
[configSol,solInfo] = ik('body7',MTH_RST,weights,initialguess);%Solucion cinematica inversa a partir de los parametros anteriores
qsolRST = struct2table(configSol);

MTHinvRST=getTransform(RkukaRST,configSol,'body7');%Verificacion MTH a partir de solucion encontrada

%% Pruebas
coordinates=[ 0.4 0.6 0.5 30 20 45;
              -0.35 0.4 0.8 40 60 10;
              0.3 0.2 -0.1 -30 180 4;
              0.4 0.5 0 30 45  10];

CR1 = coordinates( 1, 4:6);
CR1 = eul2tr(CR1);
CR1(1:3 , 4) = coordinates(1:3,1)';
% MTH = [ [0.3 0.5 0.5 0 ]' [0.3 0.3 0.4 0 ]' [0.2 0 0 0 ]' [0.2 0.2 0.3 1]' ]; 


[Qa , e1 ] = inverse_k(CR1 ,RKuka)

R1 = tr2eul(MTH(1:3,1:3));

CR1 = coordinates( 1, 4:6);
CR1 = eul2tr(CR1);
CR1(1:3 , 4) = coordinates(1:3,1)';
% MTH = [ [0.3 0.5 0.5 0 ]' [0.3 0.3 0.4 0 ]' [0.2 0 0 0 ]' [0.2 0.2 0.3 1]' ]; 


[Qa , e1 ] = inverse_k(CR1 ,RKuka)

R1 = tr2eul(MTH(1:3,1:3));



%% functions

function [q_calc,e,Tcalc]=inverse_k(MTH, RKuka)
  
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


% c3= (L1+L5*cos(q2)*c4 +L3*cos(q2)-z )/(L3*sin(q1)*s4)
% q3=atan2(sqrt(1-c3^2),c3);


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



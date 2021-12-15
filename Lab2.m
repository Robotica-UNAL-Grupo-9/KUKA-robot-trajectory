%% 6
clc, clear ,close all


L1=0.340; L3=0.400; L5=0.400; L7=0.161;


L(1) = Link('revolute'   ,'alpha',      0,  'a',  0,     'd',    L1 , 'offset',     0, 'qlim',  [-170*pi/180 170*pi/180],   'modified');
L(2) = Link('revolute'  ,'alpha',   pi/2,  'a',  0, 'd',    0 , 'offset',    0, 'qlim',       [-120*pi/180 120*pi/180],   'modified');
L(3) = Link('revolute'   ,'alpha',  -pi/2,  'a',  0,     'd',    L3 , 'offset',     0, 'qlim',  [-170*pi/180 170*pi/180],   'modified');
L(4) = Link('revolute'   ,'alpha',  pi/2,  'a',  0,     'd',    0 , 'offset',     0, 'qlim',  [-120*pi/180 120*pi/180],   'modified');
L(5) = Link('revolute'   ,'alpha',  -pi/2,  'a',  0,     'd',    L5, 'offset',     0, 'qlim',  [-170*pi/180 170*pi/180],   'modified');
L(6) = Link('revolute'   ,'alpha',  pi/2,  'a',  0,     'd',    0 , 'offset',     0, 'qlim',  [-120*pi/180 120*pi/180],   'modified');
L(7) = Link('revolute'   ,'alpha',  -pi/2,  'a',  0,     'd',    L7 , 'offset',     0, 'qlim',  [-175*pi/180 175*pi/180],   'modified');

ws=[-5 2 -4 4 -2 5];

plot_options = {'workspace',ws,'scale',.4,'view',[125 25],'jaxes','basewidth',10};
RKuka = SerialLink(L,'name','Kuka','plotopt',plot_options)

%Determinar la posición del último sistema coordenado dado q:
% q3=0
q=[pi/3 pi/6 0 pi/4 -pi/3 3*pi/4 pi/4];

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
q1=atan2(-y,-x);
q4=atan2(s4,c4);

q2=-beta+gamma;


q_calc=[q1,q2,q3,q4,0,0,0];


Twrist=RKuka.fkine(q_calc);
[Rwrist,Pwrist]=tr2rt(Twrist);

R36=inv(Rwrist)*R;

angles=tr2eul(R36,'deg')  % Phi, Theta Psi z,x,y
q_567_alt=tr2eul(R36);

c6=R36(3,3);


q5=atan2(R36(2,3),R36(1,3));
q6=atan2(sqrt(1-c6^2),c6);
q7=atan2(R36(3,2),-R36(3,1));

q_567=[q5 q6 q7] ;

q_calc=[q1,q2,q3,q4,q_567];

q*180/pi
q_calc*180/pi

Tcalc=RKuka.fkine(q_calc);

e=Tcalc -MTH

var=RKuka.fkine([q(1:4), 0 0 0]);


%% Punto 3 
close all

t=linspace(0,1,1)';

n=4;
RKuka.plot([q(1:n), zeros(1,7-n)].*t)
hold on
RKuka.plot([q_calc(1:n), zeros(1,7-n)].*t)


%% 5.2 (old)

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

%test=homeConfiguration(RkukaRST)
q=[pi/3 pi/6 pi/2 pi/4 3*pi/4 3*pi/4 3*pi/4];
qRST = struct('JointName',{'jnt1','jnt2','jnt3','jnt4','jnt5','jnt6','jnt7'},'JointPosition',{q(1),q(2),q(3),q(4),q(5),q(6),q(7)});

%motionModel = taskSpaceMotionModel("RigidBodyTree",RkukaRST)
transformRST = getTransform(RkukaRST,qRST,'body7')
clf
figure()
show(RkukaRST,qRST);




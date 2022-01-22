%% 5.1
clf
clear variables
clc
syms q1 q2 q3 q4 q5 q6 q7 MF L_1 L_2 L_3 real
MF=0.1;
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

ws=[-2 2 -2 2 -2 2];

plot_options = {'workspace',ws,'scale',.4,'view',[125 25],'basewidth',10};
RKuka = SerialLink(L4,'name','Kuka','plotopt',plot_options);

cir = circle([0.2,0], 0.2);%Trayectoria circular

x=cir(1,26:50);
y=cir(2,2:26);

for i = length(x):-1:2
    x(i)=x(i)-x(i-1);
end  
for i = length(y):-1:2
    y(i)=y(i)-y(i-1);
end

MTH1=transl(0.34,-0.2,0.34)*troty(45,'deg');%
MTH2=MTH1*transl(0,0.2,0);
MTH3=zeros(4,4,25);
MTH3(:,:,1)=MTH2*transl(x(1),y(1),0);
for i=2:length(x)
    
    MTH3(:,:,i)=MTH3(:,:,i-1)*transl(x(i),y(i),0);
    
end

% MTH34=MTH3(:,:,25)*transl(0.4,0,0);

%  MTH3=MTH2*transl(0.4,0.0,0);

 MTH4=MTH3(:,:,25)*transl(0,-0.2,0);
 MTH5=MTH4*transl(-0.4,0,0);


T_traj = ctraj(MTH1,MTH2,8); 
T_traj3 = ctraj(MTH3(:,:,25),MTH4,8); 
T_traj4 = ctraj(MTH4,MTH5,8); 

q_ctraj1 = RKuka.ikunc(T_traj(:,:,:));
q_ctraj2 = RKuka.ikunc(MTH3(:,:,:));
q_ctraj3 = RKuka.ikunc(T_traj3(:,:,:));
q_ctraj4 = RKuka.ikunc(T_traj4(:,:,:));

q_ctraj=[q_ctraj1;q_ctraj2;q_ctraj3;q_ctraj4];


%%
figure
hold on
trplot(eye(4),'rgb')
axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])
for i=1:length(q_ctraj)
    punto = RKuka.fkine(q_ctraj(i,:));
    RKuka.plot(q_ctraj(i,:))
    plot3(punto(1,4),punto(2,4),punto(3,4),'r*')
    %view(3)
    pause(0.1)
end

figure
plot(q_ctraj,'linewidth',2)
grid on
legend('q1','q2','q3','q4','q5','q6','q7')
xlabel('Paso de tiempo')

%% 
via=q_ctraj;
time=ones(1,60);
dt=0.2;
q_s= mstraj(via,[],time,ikunc(RKuka,MTH1),dt,0.5);

figure
hold on
trplot(eye(4),'rgb')

axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])

for j = 1:size(via,1)
    punto = RKuka.fkine(via(j,:));
    plot3(punto(1,4),punto(2,4),punto(3,4),'kx')
end

axis([-1.5 1.5 -1.5 1.5 -1.5 1.5])
for i=1:length(q_s)
    punto = RKuka.fkine(q_s(i,:));
    RKuka.plot(q_s(i,:))
    plot3(punto(1,4),punto(2,4),punto(3,4),'-o','LineWidth',2,...
                       'MarkerEdgeColor','k',...
                       'MarkerFaceColor','g',...
                       'MarkerSize',2)

    axis([-1 3 -1 2])
    M(i) = getframe();

end
%% Modelo diferencial
%A partir del ejemplo del Calculo del Jacobiano del Robot Stanford por
%metodos geometricos

%Matrices MTH
T01 = RKuka.A(1, [q1 q2 q3 q4 q5 q6 q7])
T12 = RKuka.A(2, [q1 q2 q3 q4 q5 q6 q7]);
T23 = RKuka.A(3, [q1 q2 q3 q4 q5 q6 q7]);
T34 = RKuka.A(4, [q1 q2 q3 q4 q5 q6 q7]);
T45 = RKuka.A(5, [q1 q2 q3 q4 q5 q6 q7]);
T56 = RKuka.A(6, [q1 q2 q3 q4 q5 q6 q7]);
T67 = RKuka.A(7, [q1 q2 q3 q4 q5 q6 q7]);

R_tool = [1 0 0; 0 1 0; 0 0 1]; P_tool = [0 0 0]';
T_4_tool = rt2tr(R_tool,P_tool);


A01 = simplify(T01);
A02 = simplify(A01*T12);
A03 = simplify(A02*T23);
A04 = simplify(A03*T34);
A05 = simplify(A04*T45);
A06 = simplify(A05*T56);
A07 = simplify(A06*T67);

A0NOA = A07*T_4_tool; %Cinematica directa
% Calculos de 0Zi
z01 = A01(1:3,3); %art 1
z02 = A02(1:3,3); %art 2
z03 = A03(1:3,3); %art 3
z04 = A04(1:3,3); %art 4
z05 = A05(1:3,3); %art 5
z06 = A06(1:3,3); %art 6
z07 = A07(1:3,3); %art 7

%Vectores Pi
p7NOA=A0NOA(1:3,4)-A07(1:3,4);
p6NOA=A0NOA(1:3,4)-A06(1:3,4);
p5NOA=A0NOA(1:3,4)-A05(1:3,4);
p4NOA=A0NOA(1:3,4)-A04(1:3,4);
p3NOA=A0NOA(1:3,4)-A03(1:3,4);
p2NOA=A0NOA(1:3,4)-A02(1:3,4);
p1NOA=A0NOA(1:3,4)-A01(1:3,4);
%Jacobianos
J1=[(skew(z01)*p1NOA); z01]; %Rotacional
J2=[(skew(z02)*p2NOA); z02]; %Rotacional
J3=[(skew(z03)*p3NOA); z03]; %Rotacional
J4=[(skew(z04)*p4NOA); z04]; %Rotacional
J5=[(skew(z05)*p5NOA); z05]; %Rotacional
J6=[(skew(z06)*p6NOA); z06]; %Rotacional
J7=[(skew(z07)*p7NOA); z07]; %Rotacional
jacob=([J1,J2,J3,J4,J5,J6,J7]); %% Matriz jacobiana a manera simbólica en función de q1,q2,q3,q4,q5 
jacob=ceros(jacob);

%%
q1=0;
q2=0;
q3=0;
q4=0;
q5=0;
q6=0;
q7=0;
jacob_solve=eval(Jacobiano)
v=[0.100 0.200 0.050]; %m/s
w=[5 10 -5];    %rad/s

dq_solve= pinv(Jacobiano_resul)*[v w]'

%% Comprobacion Jacobiano
q=[0 0 0 0 0 0 0]; %posición a analizar

jacobiano=RKuka.jacob0(q)

v=[0.100 0.200 0.050]; %m/s
w=[5 10 -5];    %rad/s

dq=pinv(jacobiano)*[v w]';

%%
function T=ceros(MTH)
    for i=1:4
        for j=1:4
            [C,T]=coeffs(MTH(i,j));
            C=round(C);
            MTH(i,j)=dot(C,T);
        end
    end
    T=MTH;
end

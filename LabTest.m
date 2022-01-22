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
MTH2=MTH1*transl(0,0.6,0);
MTH3=zeros(4,4,25);
MTH3(:,:,1)=MTH2*transl(x(1),y(1),0);
for i=2:length(x)
    
    MTH3(:,:,i)=MTH3(:,:,i-1)*transl(x(i),y(i),0);
    
end

% MTH34=MTH3(:,:,25)*transl(0.4,0,0);

%  MTH3=MTH2*transl(0.4,0.0,0);

 MTH4=MTH3(:,:,25)*transl(0,-0.6,0);
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
[R,p]=tr2rt(RKuka.fkine(q_ctraj));
Xs=p(:,1);
Ys=p(:,2);
Zs=p(:,3);
plot3(Xs,Ys,Zs);
xlabel('X') 
ylabel('Y')
zlabel('Z')
view([50,15])
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

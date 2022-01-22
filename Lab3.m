%% set up
clc, clear ,close all
% useful functions Peter Corke toolbox
%lspb linear segment parabolic bend
%jtraj 
%mtraj


rMax=1.2;

rMin=0.4;
p0=[0.5,-0.2,0.75];
L_distance=0.4*rMax/2;
t=linspace(0,1,100);

x=0.4*([0,-1,-1,1,1,0]).*L_distance;
y=([-1,-1,1,1,-1,-1]+1)*L_distance;
n=length(x);
z=zeros(1,n);
Plano=transl(p0)*troty(45,'deg')*[x;y;z;ones(1,n)]
% n=[1 0 1] -> Roty(45,'deg')
X=Plano(1,:);
Y=Plano(2,:);
Z=Plano(3,:);



qdMax=1/100*[  170;  120; 170;  120;170;  120; 175];
q0=[0 0 0 0 0 0 0 ]; 
q0=[0 0 0 ]; 

% q = invekinematic(Plano)
q=q0+Plano(1:3,:)';
%traj = mstraj(p, qdmax, tseg, q0, dt, tacc, options)
tseg=20;
qdmax=2;
dt=0.05;
tacc=0.3;     % tiempo de aceleracion
qdmax=2;

%traj=jtraj(p0,q(end,:),tseg);

%traj = mstraj(p, qdmax, tseg, q0, dt, tacc, options)
traj = mstraj(q,qdmax,[], p0,dt,tacc)


%traj=traj(1:end,:)
[x, y, z] = sphere(20)

plot3(X,Y,Z,'LineWidth',2)
hold on
plot3(traj(:,1),traj(:,2),traj(:,3), 'LineWidth',2.5)
% limite interior
surf(rMin*x,rMin*y,rMin*z+0.34)
% limite exterior
surf(rMax*x,rMax*y,rMax*z+0.34,'faceColor','none')

grid on
axis equal
xlim([-1,1])
ylim([-1,1])
zlim([-1,2])
xline(0)
yline(0)

% figure()
% plot(traj(:,1))

%% Robot DH specifications
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

plot_options = {'workspace',ws,'scale',.2,'view',[170 12],'basewidth',10,'noarrow'};
RKuka = SerialLink(L,'name','Kuka','plotopt',plot_options)

%%
close all
for k= 1:size(traj,1)
  
  MTH(:,:,k)=transl(traj(k,:))*troty(45,'deg');

end
%figure('units','normalized','outerposition',[0 0 1 1])
q_ant=[-0.4677   37.8473    0.0046 -130.2803   -0.0007   35.8844    0.4630];
q_all=zeros(7, size(traj,1) );
qikine=RKuka.ikunc(MTH);
plot(180/pi*qikine,'-','linewidth',2,'MarkerSize',5)
grid on
yline(-170,'--b')
yline(170,'--b')
yline(-120,'--r')
yline(120,'--r')



legend('q1 b','q2 r','q3 b','q4 r','q5 b','q6 r ','q7 b')
xlabel('Paso de tiempo')
%
%%
plot(RKuka,qikine(1,:))
hold on
plot3(traj(:,1),traj(:,2),traj(:,3),'.')
  xlim([-1,1])
ylim([-1,1])
zlim([-1,2])
xline(0)
yline(0)

% qikine=RKuka.ikunc(MTH(:,:,1));
%  plot(RKuka,qikine)
%  hold on
%  plot3(traj(1:k,1),traj(1:k,2),traj(1:k,3),'.')
%  hold off
%% 
figure()


for k=1:size(traj,1)
  
  plot(RKuka,qikine(k,:))
  hold on
  plot3(traj(1:k,1),traj(1:k,2),traj(1:k,3),'.')
  axis equal
  xlim([-1,1])
  ylim([-1,1])
  zlim([-1,2])
  xline(0)
  yline(0)
  % limite interior
  surf(rMin*x,rMin*y,rMin*z+0.34)
  % limite exterior
  surf(rMax*x,rMax*y,rMax*z+0.34,'faceColor','none')
  
  hold off
  pause(0.1)
end






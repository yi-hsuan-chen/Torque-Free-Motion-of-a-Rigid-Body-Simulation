% =======================================================================
% Author:       Yi-Hsuan Chen, University of Maryland, College Park
% Date:         06-July-2024
% Description:  This code demonstrates the distinct behaviors of torque-free rotation
%               of prolate and oblate objects. The visualization also illustrates space 
%               and body cones, showing the rotational behavior of the 
%               body in the inertial and body-fixed frames.
% Youtube link: https://youtu.be/YyfLiDVZ8VY?si=y95bNCZBMFow6fiG
% =======================================================================

clc; clear; close all;

%% User-defined params
config      = 'p';          % p: prolate object, o: oblate object
record      = 0;            % 1: record the video, 0: not record the video
divider     = 50;           % Increase this to speed up the animation


%... numerical integration settings
tfinal      = 10;
dt          = 1e-3;

switch config
    case 'p'
        w           = 0.5;
        h           = 2;
        wo          = 1;
        w3          = 5;
    case 'o'
        w           = 4;
        h           = 0.5;
        wo          = 4;
        w3          = 5;
end

m           = 1;
IT          = 1/12*m*(h^2+w^2);
I3          = 1/6*m*w^2;
wn          = (1-I3/IT)*w3;
theta       = acos(I3*w3/sqrt((IT*wo)^2+(I3*w3)^2));
para.IT     = IT;
para.I3     = I3;
para.wo     = wo;
para.w3     = w3;
para.theta  = theta;

X0          = [0;0];
tspan       = 0:dt:tfinal;
opts        = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t,y]       = ode45(@(t,x) torqueFreeMotionSymmeticTop(x,para), tspan, X0, opts);
psi         = y(:,1);
phi         = y(:,2);

tt          = 0:1e-3:tfinal;
Psi         = interp1(t,psi,tt);
Phi         = interp1(t,phi,tt);

%% Plot the axes hG (e3), I_w_B, b3
% Define unit vector hG describing e3-axis of the 'inertia' frame
qlw         = 2; 
len         = 2;
e1          = [len;0;0];
e2          = [0;len;0];
hG          = [0;0;3*len];

fig(1) = figure(); ax(1) = gca;
p(1)        = quiver3(0,0,0,e1(1),e1(2),e1(3),'r--','LineWidth',.5*qlw); hold on; 
p(2)        = quiver3(0,0,0,e2(1),e2(2),e2(3),'r--','LineWidth',.5*qlw);
p(3)        = quiver3(0,0,0,hG(1),hG(2),hG(3),'r-','LineWidth',qlw);
xlabel('$x$'); ylabel('$y$'); zlabel('$z$');
axis equal; view(3); rotate3d on; grid on;
lim         = 6; 
xlim([-lim lim]); ylim([-lim lim]); zlim([-lim/2 lim]); 

%% Function handle of rotation matrices (3-2-3) psi-theta-phi
C_C_B = @(phi) [ cos(phi)   -sin(phi)     0;
                 sin(phi)   cos(phi)     0;
                   0        0        1];
             
A_C_C = @(th) [ cos(th)     0     sin(th);
                   0        1        0;
               -sin(th)     0     cos(th)];       
           
I_C_A = @(psi) [ cos(psi)   -sin(psi)     0;
                 sin(psi)   cos(psi)     0;
                   0        0        1];
%% Inertial and body framed initialization
cCB         = C_C_B(0);
aCc         = A_C_C(theta);
ICa         = I_C_A(0);
I_C_B       = ICa*aCc*cCB;
DCM         = I_C_B;
b1          = DCM*e1;
b2          = DCM*e2;
b3          = DCM*hG;

p(4)        = quiver3(0,0,0,b1(1),b1(2),b1(3),'b--','LineWidth',.5*qlw); 
p(5)        = quiver3(0,0,0,b2(1),b2(2),b2(3),'b--','LineWidth',.5*qlw);
p(6)        = quiver3(0,0,0,b3(1),b3(2),b3(3),'b-','LineWidth',qlw); 
IwB         = DCM*[-wo;0;w3];
p(7)        = quiver3(0,0,0,IwB(1),IwB(2),IwB(3),'g-','LineWidth',2*qlw); 

%% Plot tht 3D box
x           = w/2;
y           = x;
z           = h/2;
vv          = [x -y -z; x y -z; -x y -z; -x -y -z;
               x -y z; x y z; -x y z; -x -y z];
fac         = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
VV          = DCM*vv';
p(8)        = patch('Vertices',VV','Faces',fac,'FaceColor',[0.8 0.8 0.8],'FaceAlpha',0.2);

%% Patch space and body cones
% Define range of angles to rotate through
mu          = linspace(0,2*pi,200);
f           = 5;            % scaling factor

% space cone: Calculate resulting new vector v  
% (Rotate IwB/u around vector hG/n)
n       = (1/norm(hG))*hG;
u       = (1/norm(IwB))*IwB;
v       = (1-cos(mu))*dot(n,u).*n + cos(mu).*u - sin(mu).*cross(n,u);
f0      = [1 2];
vertice = f*[0 0 0; u';v(:,1:end)'];
face    = 1:(length(mu)+length(f0));
p(9)    = patch('Faces',face,'Vertices',vertice,'FaceColor','magenta','FaceAlpha',0.3);

% body cone: Calculate resulting new vector v (time-varying)
% (Rotate IwB/u around vector b3/n)
n       = (1/norm(b3))*b3;
v       = (1-cos(mu))*dot(n,u).*n + cos(mu).*u - sin(mu).*cross(n,u);
vertice = f*[0 0 0; u';v(:,1:end)'];
f0      = [1 2];
face    = 1:(length(mu)+length(f0));
p(10)    = patch('Faces',face,'Vertices',vertice,'FaceColor','blue','FaceAlpha',0.2);


set(ax,'Fontsize',12,'XGrid','on','YGrid','on','TickLabelInterpreter','latex');
for i = 1:length(ax)
    set(ax(i).XLabel,'Interpreter','latex');
    set(ax(i).YLabel,'Interpreter','latex');
    set(ax(i).ZLabel,'Interpreter','latex');
end

%% animation
if record
    v_handle = VideoWriter(Filename,'MPEG-4');
    open(v_handle);
end
RT      = zeros(4,4);
for i = 1:divider:length(tt)
    % Compute rotation matrix Cbv
    cCB         = C_C_B(Phi(i));
    aCc         = A_C_C(theta);
    ICa         = I_C_A(Psi(i));
    I_C_B       = ICa*aCc*cCB;
    DCM         = I_C_B;
    b1          = DCM*e1;
    b2          = DCM*e2;
    b3          = DCM*hG;
    
    w1          = -wo*cos(wn*tt(i));
    w2          = wo*sin(wn*tt(i));
    IwB         = DCM*[w1;w2;w3];

    %% update rotating box
    VV          = DCM*vv';
    set(p(8),'Faces',fac,'Vertices',VV','FaceColor','black');
    
    %% body cone
    n           = (1/norm(b3))*b3;
    u           = (1/norm(IwB))*IwB;
    v           = (1-cos(mu))*dot(n,u).*n + cos(mu).*u - sin(mu).*cross(n,u);
    vertice     = f*[0 0 0; u';v(:,1:end)'];
    set(p(10),'Faces',face,'Vertices',vertice,'FaceColor','cyan','FaceAlpha',0.3)
    
    set(p(4),'XData',0,'YData',0,'ZData',0,'UData',b1(1),'VData',b1(2),'WData',b1(3));
    set(p(5),'XData',0,'YData',0,'ZData',0,'UData',b2(1),'VData',b2(2),'WData',b2(3));
    set(p(6),'XData',0,'YData',0,'ZData',0,'UData',b3(1),'VData',b3(2),'WData',b3(3));
    set(p(7),'XData',0,'YData',0,'ZData',0,'UData',IwB(1),'VData',IwB(2),'WData',IwB(3));
    drawnow;
    % Get animation frame
    if record
        frame = getframe(gcf);
        writeVideo(v_handle,frame);
    end
end
if record
    close(v_handle);
end

function dX = torqueFreeMotionSymmeticTop(X,para)
wo      = para.wo;
w3      = para.w3;
IT      = para.IT;
I3      = para.I3;
theta   = para.theta;

wn      = (1-I3/IT)*w3;
psi     = X(1);
phi     = X(2);
dX      = zeros(2,1);
dX(1)   = wo/sin(theta);
dX(2)   = wn;
end
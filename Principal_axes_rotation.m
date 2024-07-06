% =======================================================================
% Author:       Yi-Hsuan Chen, University of Maryland, College Park
% Date:         06-July-2024
% Description:  This code demonstrates the principal axes theorem by 
%               simulating the motion of a torque-free rigid body. The 
%               visualization illustrates the stability of rotations 
%               about different principal axes.
% Youtube link: https://youtu.be/YyfLiDVZ8VY?si=UfEEBARv_RK7O31-
% =======================================================================

clc; clear; close all;

%% User-defined params
AXES    = 'int';        % rotate about 'max','min' (stable), 'int' (unstable)
record  = 0;            % 1: record the video, 0: not record the video


%... numerical integration settings
tfinal  = 10;
dt      = 1e-3;

%%... system parameters
m       = 1;
d       = 8;
w       = 4;
h       = 0.5;
I1      = m/12*(h^2+w^2);
I2      = m/12*(h^2+d^2);
I3      = m/12*(d^2+w^2);
para.I1 = I1;
para.I2 = I2;
para.I3 = I3;

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

%% Plot inertia axes e1, e2, e3
phi0        = 0;
theta0      = -pi/2;
psi0        = 0;
% Compute rotation matrix Cbv
cCB         = C_C_B(phi0);
aCc         = A_C_C(theta0);
ICa         = I_C_A(psi0);
I_C_B       = ICa*aCc*cCB;
DCM         = I_C_B;

qlw         = 2;
len         = 5;
e1          = [len;0;0];
e2          = [0;len;0];
e3          = [0;0;len];
e10         = DCM*e1;
e20         = DCM*e2;
e30         = DCM*e3;

fig(1) = figure(); ax(1) = gca;
p(1)        = quiver3(0,0,0,e10(1),e10(2),e10(3),'r-','LineWidth',qlw); hold on;
p(2)        = quiver3(0,0,0,e20(1),e20(2),e20(3),'g-','LineWidth',qlw);
p(3)        = quiver3(0,0,0,e30(1),e30(2),e30(3),'b-','LineWidth',qlw);
xlabel('$x$  (m)'); ylabel('$y$  (m)'); zlabel('$z$  (m)');
axis equal; view([50 30]); rotate3d on; grid on;
lim         = 6; 
xlim([-lim lim]); ylim([-lim lim]); zlim([-lim lim]); 

%% Plot tht 3D box
x           = d/2;
y           = w/2;
z           = h/2;
vv          = [x -y -z; x y -z; -x y -z; -x -y -z;
               x -y z; x y z; -x y z; -x -y z];
fac         = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
p(4)        = patch('Vertices',(DCM*vv')','Faces',fac,'FaceColor',[0.8 0.8 0.8],'FaceAlpha',0.2);

%% Numerical integration
switch AXES
    case 'max'
        w0          = [0.001;0;10];
    case 'int'
        w0          = [0.1;10;0.1];
    case 'min'
        w0          = [10;0.001;0.001];
end
X0          = [w0;phi0;theta0;psi0];

tspan       = 0:dt:tfinal;
[t,X]       = ode45(@(t,x) torqueFreeMotion(x,para), tspan, X0);

psi         = X(:,4);
theta       = X(:,5);
phi         = X(:,6);

set(ax,'Fontsize',12,'XGrid','on','YGrid','on','TickLabelInterpreter','latex');
for i = 1:length(ax)
    set(ax(i).XLabel,'Interpreter','latex');
    set(ax(i).YLabel,'Interpreter','latex');
    set(ax(i).ZLabel,'Interpreter','latex');
end

%% animation
Filename = horzcat(AXES,'_axis_rotation');
if record
    v_handle = VideoWriter(Filename,'MPEG-4');
    open(v_handle);
end
divider = 50;
NN      = length(t);
for i = 1:divider:NN
    % Compute rotation matrix Cbv
    cCB         = C_C_B(phi(i));
    aCc         = A_C_C(theta(i));
    ICa         = I_C_A(psi(i));
    I_C_B       = ICa*aCc*cCB;
    DCM         = I_C_B;
    b1          = DCM*e1;
    b2          = DCM*e2;
    b3          = DCM*e3;
    
    %%... update body axis    
    set(p(1),'XData',0,'YData',0,'ZData',0,'UData',b1(1),'VData',b1(2),'WData',b1(3));
    set(p(2),'XData',0,'YData',0,'ZData',0,'UData',b2(1),'VData',b2(2),'WData',b2(3));
    set(p(3),'XData',0,'YData',0,'ZData',0,'UData',b3(1),'VData',b3(2),'WData',b3(3));

    %%... update rotating box
    VV          = DCM*vv';
    set(p(4),'Faces',fac,'Vertices',VV','FaceColor','black');
   
    drawnow;
    if i> NN/4 && (i<NN*2/4)
        view([0 90]);
    else
        view([150 30]);
    end
    % Get animation frame
    if record
        frame = getframe(gcf);
        writeVideo(v_handle,frame);
    end
end
if record
    close(v_handle);
end


%%... Equation of Torque-Free Motion
function dX = torqueFreeMotion(X,para)
I1      = para.I1;
I2      = para.I2;
I3      = para.I3;

w1      = X(1);
w2      = X(2);
w3      = X(3);
psi     = X(4);
theta   = X(5);
phi     = X(6);
dX      = zeros(6,1);
dX(1)   = w2*w3*(I2-I3)/I1;
dX(2)   = w3*w1*(I3-I1)/I2;
dX(3)   = w1*w2*(I1-I2)/I3;
dX(4)   = (-w1*cos(phi)+w2*sin(phi))*csc(theta);
dX(5)   = w1*sin(phi)+w2*cos(phi);
dX(6)   = (w1*cos(phi)-w2*sin(phi))*cot(theta)+w3;
end
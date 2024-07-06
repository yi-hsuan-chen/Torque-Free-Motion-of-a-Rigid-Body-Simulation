% =======================================================================
% Author:       Yi-Hsuan Chen, University of Maryland, College Park
% Date:         27-Dec-2023
% Description:  The code is intended to demonstrate and visualize the 
%               motion of a torque-free rigid body and determine its 
%               trajectory in the inertial frame. This example is derived 
%               from a YouTube tutorial by Dr. Shane Ross.
% Reference:    Dual-Spin Spacecraft, Stabilizing Rotation About Any Principal 
%               Axis (https://youtu.be/8uOxYf9nLNw?si=lk6x0mVaOwIGljrr)
% =======================================================================

clc; clear; close all;

%... user-defined flag
record      = 0;
controlFlag = 1;        % 1/0: fly-wheel on/off
                        % the satellite will flip around when the fly-wheel
                        % is off due to the Intermediate Axis Theorem.

%... system parameters
if controlFlag
    w_hat   = 20;
else
    w_hat   = 0;
end
I1          = 350;
I2          = 300;
I3          = 400;
Iws         = 10;
we1         = 60*2*pi/60;
Filename    = horzcat('Omega_',num2str(w_hat),'_rotation');

para.I1     = I1;
para.I2     = I2;
para.I3     = I3;
para.Iws    = Iws;
para.we1    = we1;
para.what   = w_hat;

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

%% Initial settings for inertia axes e1, e2, e3
phi0        = 0;
theta0      = -pi/2;
psi0        = 0;

% Compute rotation matrix Cbv
cCB         = C_C_B(phi0);
aCc         = A_C_C(theta0);
ICa         = I_C_A(psi0);
I_C_B       = ICa*aCc*cCB;
DCM0        = I_C_B;

qlw         = 2;
len         = 10;
e1          = [len;0;0];
e2          = [0;len;0];
e3          = [0;0;len];
e10         = DCM0*e1;
e20         = DCM0*e2;
e30         = DCM0*e3;

%... plot the inertial coordinate
fig(1) = figure(); ax(1) = gca;
p(1)        = quiver3(0,0,0,e10(1),e10(2),e10(3),'r-','LineWidth',qlw); hold on;
p(2)        = quiver3(0,0,0,e20(1),e20(2),e20(3),'g-','LineWidth',qlw);
p(3)        = quiver3(0,0,0,e30(1),e30(2),e30(3),'b-','LineWidth',qlw);
xlabel('$x$  (m)'); ylabel('$y$  (m)'); zlabel('$z$  (m)');
axis equal; view([50 30]); rotate3d on; grid on;
lim         = 15; 
xlim([-lim lim]); ylim([-lim lim]); zlim([-lim lim]); 

%... plot the 3D box
x           = 5;
y           = 10;
z           = 4;
vv          = [x -y -z; x y -z; -x y -z; -x -y -z;
               x -y z; x y z; -x y z; -x -y z];
fac         = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
p(4)        = patch('Vertices',(DCM0*vv')','Faces',fac,'FaceColor',[0.8 0.8 0.8],'FaceAlpha',0.2);

%... plot the flywheel
b_C_w       = @(gamma) [ 1 0 0;0 cos(gamma) sin(gamma);0 -sin(gamma) cos(gamma)];
               
r           = 3;
[X,Y,Z]     = cylinder(r,50);
h           = 2;
Z           = Z*h-h/2;
positionOld1 = [X(1,:)',Y(1,:)',Z(1,:)'];
positionOld2 = [X(2,:)',Y(2,:)',Z(2,:)'];

p(5)        = surf(X,Y,Z,'FaceColor','cyan','FaceAlpha',0.5);

th          = 0:pi/50:2*pi;
xunit       = r*cos(th);
yunit       = r*sin(th);
top         = [r*cos(th);r*sin(th);h/2*ones(1,length(yunit))];
bottom      = [r*cos(th);r*sin(th);-h/2*ones(1,length(yunit))];
p(6)        = fill3(xunit,yunit,h/2*ones(1,length(yunit)),'cyan','FaceAlpha',0.5);
p(7)        = fill3(xunit,yunit,-h/2*ones(1,length(yunit)),'cyan','FaceAlpha',0.5);

aa          = 0.5;
wb01        = e10;
wb02        = aa*e20;
wb03        = aa*e30;
p(8)        = quiver3(0,0,0,wb01(1),wb01(2),wb01(3),'m-','LineWidth',2*qlw); hold on; 
p(9)        = quiver3(0,0,0,wb02(1),wb02(2),wb02(3),'k-','LineWidth',qlw);
p(10)       = quiver3(0,0,0,wb03(1),wb03(2),wb03(3),'k-','LineWidth',qlw);

%% Numerical integration
we0         = [we1;0;0];
dw0         = [0;0.1;0.1];
w0          = we0+dw0;
X0          = [w0;phi0;theta0;psi0;0];
tf          = 30;
tspan       = [0 tf];
[t,X]       = rk4(@(t,x) dualspin(t,x,para), tspan, X0, 1e-4);

psi         = X(:,4);
theta       = X(:,5);
phi         = X(:,6);
gamma       = X(:,7);

set(ax,'Fontsize',12,'XGrid','on','YGrid','on');
set(ax,'TickLabelInterpreter','latex');
for i = 1:length(ax)
    set(ax(i).XLabel,'Interpreter','latex');
    set(ax(i).YLabel,'Interpreter','latex');
    set(ax(i).ZLabel,'Interpreter','latex');
end


%% Visualization
if record
    v_handle = VideoWriter(Filename,'MPEG-4');
    open(v_handle);
end
divider = 100;
NN      = length(t);
for i = 1:divider:NN
    %... Compute rotation matrix Cbv
    cCB         = C_C_B(phi(i));
    aCc         = A_C_C(theta(i));
    ICa         = I_C_A(psi(i));
    I_C_B       = ICa*aCc*cCB;
    DCM         = I_C_B;
    b1          = DCM*e1;
    b2          = DCM*e2;
    b3          = DCM*e3;
    
    %... update body axes    
    set(p(1),'XData',0,'YData',0,'ZData',0,'UData',b1(1),'VData',b1(2),'WData',b1(3));
    set(p(2),'XData',0,'YData',0,'ZData',0,'UData',b2(1),'VData',b2(2),'WData',b2(3));
    set(p(3),'XData',0,'YData',0,'ZData',0,'UData',b3(1),'VData',b3(2),'WData',b3(3));

    %... update rotating box
    VV          = DCM*vv';
    set(p(4),'Faces',fac,'Vertices',VV','FaceColor','black');
    
    %% update wheel axes
    bRw         = b_C_w(gamma(i));
    wb1         = aa*DCM*bRw*e1;
    wb2         = aa*DCM*bRw*e2;
    wb3         = aa*DCM*bRw*e3;
    set(p(8),'XData',0,'YData',0,'ZData',0,'UData',wb1(1),'VData',wb1(2),'WData',wb1(3));
    set(p(9),'XData',0,'YData',0,'ZData',0,'UData',wb2(1),'VData',wb2(2),'WData',wb2(3));
    set(p(10),'XData',0,'YData',0,'ZData',0,'UData',wb3(1),'VData',wb3(2),'WData',wb3(3));
    
    %% update rotating wheel!!
    % get points at the two rings and rotate them separately:
    positionNew1 = (DCM*bRw*DCM0*positionOld1')';
    positionNew2 = (DCM*bRw*DCM0*positionOld2')';    
    % reassemble the two sets of points into X Y Z format:
    XX          = [positionNew1(:,1),positionNew2(:,1)];
    YY          = [positionNew1(:,2),positionNew2(:,2)];
    ZZ          = [positionNew1(:,3),positionNew2(:,3)];
    set(p(5),'XData',XX,'YData',YY,'ZData',ZZ);
    
    topNew      = (DCM*bRw*DCM0*top)';
    bottomNew   = (DCM*bRw*DCM0*bottom)';
    set(p(6),'XData',topNew(:,1),'YData',topNew(:,2),'ZData',topNew(:,3));
    set(p(7),'XData',bottomNew(:,1),'YData',bottomNew(:,2),'ZData',bottomNew(:,3));
    drawnow; 
    
    if i> NN/4 && (i<NN*1/3)
        view([0 90]);
    elseif i> NN*1/3 && (i<NN*3/4)
        view([35 15]);
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



%% System dynamic model
function dX = dualspin(t,X,para)
I1      = para.I1;
I2      = para.I2;
I3      = para.I3;
Iws     = para.Iws;
we1     = para.we1;
what    = para.what;
Omega   = what*we1;

w1      = X(1);
w2      = X(2);
w3      = X(3);
psi     = X(4);
theta   = X(5);
phi     = X(6);
gamma   = X(7);

dX      = zeros(7,1);
dX(1)   = w2*w3*(I2-I3)/I1;
dX(2)   = w3*w1*(I3-I1)/I2 - Iws/I2*w3*Omega;
dX(3)   = w1*w2*(I1-I2)/I3 + Iws/I3*w2*Omega;
dX(4)   = (-w1*cos(phi)+w2*sin(phi))*csc(theta);
dX(5)   = w1*sin(phi)+w2*cos(phi);
dX(6)   = (w1*cos(phi)-w2*sin(phi))*cot(theta)+w3;
dX(7)   = Omega;
end

%% Runge–Kutta Numerical integration
function [t,y] = rk4(odefun,tspan,y0,varargin)
%... 2020/12/28 by Yi-Hsuan Chen
%... adopted from Johhnee Lee RK4
switch nargin
    case 3 
        h = 1e-3; % default step size
    case 4 
        h = varargin{1};
end

t0 = tspan(1);
tf = tspan(2);

% rk4...
t = t0:h:tf;
n = size(y0,1); % dimension of vector y
m = length(t);

y = zeros(n,m);
y(:,1) = y0;

for i = 1:m-1
    ti = t(i);
    yi = y(:,i);
    F1 = odefun(ti,yi)*h;
    F2 = odefun(ti+h/2,yi+F1/2)*h;
    F3 = odefun(ti+h/2,yi+F2/2)*h;
    F4 = odefun(ti+h,yi+F3)*h;
    y(:,i+1) = yi + 1/6*(F1 + 2*F2 + 2*F3 + F4);
end
y = y';
end



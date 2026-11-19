function varargout = Load_dynamic(arg)
% LOAD4_SFUN Rigid payload and four taut, massless, inextensible cables.
% Use a Simulink "Level-2 MATLAB S-Function" block:
%   S-function name: load4_sfun     Parameters: P
% Obtain illustrative parameters and consistent hover inputs with:
%   [P,pD0,vD0,aDfree0] = load4_sfun('defaults');
% Run the built-in numerical checks in MATLAB with:
%   load4_sfun('selftest');
%
% INPUTS (double, real, column vectors):
%  1 pD      12x1 [p1;p2;p3;p4], drone positions, world frame [m].
%  2 vD      12x1 [v1;v2;v3;v4], drone velocities, world frame [m/s].
%  3 aDfree  12x1 drone accelerations EXCLUDING cable forces [m/s^2].
%    aDfree_i = g*ez - (F_i/m_i)*R_i*ez, for thrust along -body-z.
%    Include any other known non-cable external force divided by m_i.
%    Do NOT feed actual drone acceleration into this port.
%
% OUTPUTS:
%  1 pL       3x1 payload position, world frame [m].
%  2 T        4x1 cable tensions [N], order matches pD and P.r columns.
%  3 fD      12x1 [fCable1;...;fCable4], world frame [N].
%  4 xL      12x1 [pL;vL;roll,pitch,yaw (column);omegaL].
% Drone translational blocks must use aD_i = aDfree_i + fCable_i/m_i.
%
% CONVENTIONS: world +z downward; R = Rz(yaw)*Ry(pitch)*Rx(roll)
% maps body to world. omegaL is BODY angular velocity. Angles in radians.
% Cables attach to drone centers of mass, so produce no drone body torque.
% No wind, payload actuation, cable mass, elasticity, or slack transitions.
% Initial cable position/velocity constraints must be consistent.
% Negative T means the taut-cable assumption has failed: stop the run and
% use a slack/taut model; never silently clamp T in this constrained model.
% Euler coordinates require abs(cos(pitch)) > 0.
%
% P fields: mL [kg], JL 3x3 body inertia [kg*m^2], mD 4x1 [kg],
% r 3x4 payload attachment offsets [m], ell 4x1 lengths [m],
% x0 12x1 initial load state, g [m/s^2], wn [1/s], zeta [-].
% wn=0 gives the ideal acceleration constraint. wn>0 adds numerical
% Baumgarte correction cddot+2*zeta*wn*cdot+wn^2*c=0, NOT a controller.
% Continuous states are owned by Simulink: no persistent time integration.
% Intended for normal-mode simulation; no generated-code/TLC implementation.

if ischar(arg) || isstring(arg)
    switch char(arg)
        case 'defaults'
            [P,pD,vD,a0] = defaults();
            result = {P,pD,vD,a0};
            varargout = result(1:nargout);
        case 'selftest'
            selftest();
        otherwise
            error('load4_sfun:Command','Use defaults or selftest.');
    end
    return;
end
setup(arg);
end

function setup(block)
block.NumDialogPrms = 1;
checkParameters(block);
block.DialogPrmsTunable = {'Nontunable'};
block.NumInputPorts = 3;
block.NumOutputPorts = 2;
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;
for k = 1:3
    block.InputPort(k).Dimensions = 12;
    block.InputPort(k).DatatypeID = 0;
    block.InputPort(k).Complexity = 'Real';
    block.InputPort(k).DirectFeedthrough = true;
end
outSizes = [3 4 12 12];
for k = 1:2
    block.OutputPort(k).Dimensions = 12;
    block.OutputPort(k).DatatypeID = 0;
    block.OutputPort(k).Complexity = 'Real';
end
block.NumContStates = 12;
block.SampleTimes = [0 0];
block.SimStateCompliance = 'DefaultSimState';
block.RegBlockMethod('CheckParameters', @checkParameters);
block.RegBlockMethod('InitializeConditions', @initialize);
block.RegBlockMethod('Outputs', @outputs);
block.RegBlockMethod('Derivatives', @derivatives);
end

function checkParameters(block)
P = block.DialogPrm(1).Data;
assert(isstruct(P) && isscalar(P),'P must be a scalar structure.');
names = {'mL','JL','mD','r','ell','x0','g','wn','zeta'};
for k = 1:numel(names)
    assert(isfield(P,names{k}),['Missing P.' names{k}]);
end
validateattributes(P.mL,{'double'},{'real','finite','scalar','positive'});
validateattributes(P.JL,{'double'},{'real','finite','size',[3 3]});
assert(norm(P.JL-P.JL.','fro') < 1e-12*max(1,norm(P.JL,'fro')),...
    'P.JL must be symmetric.');
[~,flag] = chol(P.JL);
assert(flag == 0,'P.JL must be positive definite.');
validateattributes(P.mD,{'double'},{'real','finite','size',[4 1],'positive'});
validateattributes(P.r,{'double'},{'real','finite','size',[3 4]});
validateattributes(P.ell,{'double'},{'real','finite','size',[4 1],'positive'});
validateattributes(P.x0,{'double'},{'real','finite','size',[12 1]});
validateattributes(P.g,{'double'},{'real','finite','scalar','positive'});
validateattributes(P.wn,{'double'},{'real','finite','scalar','nonnegative'});
validateattributes(P.zeta,{'double'},{'real','finite','scalar','nonnegative'});
end

function initialize(block)
P = block.DialogPrm(1).Data;
block.ContStates.Data = P.x0;
end

function outputs(block)
P = block.DialogPrm(1).Data;
x = block.ContStates.Data;
[~,T,fD] = mechanics(x,block.InputPort(1).Data,...
    block.InputPort(2).Data,block.InputPort(3).Data,P);
block.OutputPort(1).Data = x(1:3);
block.OutputPort(2).Data = T;
block.OutputPort(3).Data = fD;
block.OutputPort(4).Data = x;
end

function derivatives(block)
P = block.DialogPrm(1).Data;
x = block.ContStates.Data;
dx = mechanics(x,block.InputPort(1).Data,...
    block.InputPort(2).Data,block.InputPort(3).Data,P);
block.Derivatives.Data = dx;
end

function [dx,T,fD,c,cdot] = mechanics(x,pDvec,vDvec,a0vec,P)
pD = reshape(pDvec,3,4);
vD = reshape(vDvec,3,4);
a0 = reshape(a0vec,3,4);
pL = x(1:3); vL = x(4:6);
phi = x(7:9); w = x(10:12);
R = rotationZYX(phi);

% Generalized load acceleration is y=[aL;omegaDotL].
% B_i=[s_i', -s_i'*R*skew(r_i)], s_i=pL+R*r_i-pD_i.
B = zeros(4,6); sAll = zeros(3,4);
rhs = zeros(4,1); D = zeros(4,4);
c = zeros(4,1); cdot = zeros(4,1);
for i = 1:4
    ri = P.r(:,i);
    s = pL + R*ri - pD(:,i);
    if norm(s) < 1e-9
        error('load4_sfun:Geometry','Drone coincides with its attachment.');
    end
    sd = vL + R*cross(w,ri) - vD(:,i);
    c(i) = 0.5*(s.'*s - P.ell(i)^2);
    cdot(i) = s.'*sd;
    b = sd.'*sd + s.'*R*cross(w,cross(w,ri));
    correction = 2*P.zeta*P.wn*cdot(i) + P.wn^2*c(i);
    B(i,:) = [s.', -s.'*R*skew(ri)];
    D(i,i) = (s.'*s)/P.mD(i);
    rhs(i) = s.'*a0(:,i) - b - correction;
    sAll(:,i) = s;
end

% Drone dynamics were eliminated exactly using
% aD_i = a0_i + s_i*lambda_i/mD_i.
% Load dynamics: ML*y = QL - B'*lambda.
% Cable constraint: B*y - D*lambda = rhs.
ML = blkdiag(P.mL*eye(3), P.JL);
QL = [P.mL*[0;0;P.g]; -cross(w,P.JL*w)];
K = [ML, B.'; B, -D];
sol = K \ [QL;rhs];
y = sol(1:6); lambda = sol(7:10);

% Physical tension magnitude uses CURRENT length. At exact constraint
% norm(s_i)=ell_i, so T_i=ell_i*lambda_i as in the ideal derivation.
f = zeros(3,4); T = zeros(4,1);
for i = 1:4
    f(:,i) = sAll(:,i)*lambda(i);
    T(i) = norm(sAll(:,i))*lambda(i);
end
fD = f(:);

roll = phi(1); pitch = phi(2);
if abs(cos(pitch)) < 1e-6
    error('load4_sfun:EulerSingularity','Use quaternion load attitude here.');
end
Gamma = [1, sin(roll)*tan(pitch), cos(roll)*tan(pitch);...
         0, cos(roll),           -sin(roll);...
         0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)];
dx = [vL; y(1:3); Gamma*w; y(4:6)];
end

function R = rotationZYX(phi)
cr=cos(phi(1)); sr=sin(phi(1));
ct=cos(phi(2)); st=sin(phi(2));
cy=cos(phi(3)); sy=sin(phi(3));
R = [cy*ct, cy*st*sr-sy*cr, cy*st*cr+sy*sr;...
     sy*ct, sy*st*sr+cy*cr, sy*st*cr-cy*sr;...
     -st,   ct*sr,          ct*cr];
end

function S = skew(a)
S = [0,-a(3),a(2); a(3),0,-a(1); -a(2),a(1),0];
end

function [P,pD0,vD0,a0] = defaults()
% Illustrative values for plant verification, not identified paper data.
P.mL = 2;
P.JL = diag([0.20 0.20 0.30]);
P.mD = ones(4,1);
P.r = [0.5,-0.5,-0.5,0.5; 0.5,0.5,-0.5,-0.5; 0,0,0,0];
P.ell = 2.5*ones(4,1);
P.x0 = zeros(12,1);
P.g = 9.81;
P.wn = 5; P.zeta = 1;
pD = P.r - [0;0;1]*P.ell.';
pD0 = pD(:); vD0 = zeros(12,1);
Fhover = P.mD*P.g + (P.mL*P.g/4)*ones(4,1);
a = zeros(3,4);
for i=1:4
    a(:,i) = [0;0;P.g-Fhover(i)/P.mD(i)];
end
a0 = a(:);
end

function selftest()
[P,pD,vD,a0] = defaults();
[dx,T,fD,c,cd] = mechanics(P.x0,pD,vD,a0,P);
assert(norm(dx,inf)<1e-10,'Hover load acceleration failed.');
assert(norm(T-P.mL*P.g/4,inf)<1e-10,'Hover tension failed.');
aD = reshape(a0,3,4) + reshape(fD,3,4)./P.mD.';
assert(norm(aD,'fro')<1e-10,'Hover drone acceleration failed.');
assert(norm(c,inf)<1e-12 && norm(cd,inf)<1e-12,...
    'Initial cable constraints failed.');
[dx,T] = mechanics(P.x0,pD,vD,repmat([0;0;P.g],4,1),P);
assert(norm(T,inf)<1e-10,'Freefall tension failed.');
assert(norm(dx(4:6)-[0;0;P.g],inf)<1e-10,'Freefall failed.');
fprintf('PASS: hover, drone force balance, constraints, freefall.\n');
end

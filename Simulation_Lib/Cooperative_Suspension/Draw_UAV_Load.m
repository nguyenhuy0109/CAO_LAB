function Draw_UAV_Load(block)
% DRAW_UAV_LOAD 3-D visualization sink for four drones and a rigid payload.
% Use a Level-2 MATLAB S-Function block in NORMAL simulation mode.
% S-function name: Draw_UAV_Load    Parameters: leave EMPTY.
%
% INPUTS (double real column vectors):
%  1 pD    12x1 [p1;p2;p3;p4], world positions [m].
%  2 phiD  12x1 [phi1;phi2;phi3;phi4], each [roll;pitch;yaw] [rad].
%  3 xL    12x1 [pL;vL;rollL;pitchL;yawL;omegaL], from load output 4.
%  4 r     12x1 [r1;r2;r3;r4], attachment offsets in LOAD BODY frame [m].
%              Connect a Constant with value P.r(:).
% No output ports; no plant state integration or dynamics modification.
% Coordinate convention matches load4_sfun / renamed Load_dynamic:
% world +z DOWN; R=Rz(yaw)*Ry(pitch)*Rx(roll), body to world.
% Graphics use ZDir='reverse': no extra coordinate sign changes are needed.
% Cables are visual straight segments; this does not simulate sag or slack.
% Run in Normal mode. No code generation or SimState restore is provided.
% Closing the figure disables graphics for the remainder of that run.
% Restart simulation to create a new figure.
%
% EDIT settings() below to change refresh period, view, sizes and trails.

C = settings();
block.NumDialogPrms = 0;
block.NumInputPorts = 4;
block.NumOutputPorts = 0;
block.SetPreCompInpPortInfoToDynamic;
for i = 1:4
    block.InputPort(i).Dimensions = 12;
    block.InputPort(i).DatatypeID = 0;
    block.InputPort(i).Complexity = 'Real';
    block.InputPort(i).DirectFeedthrough = true;
end
block.NumContStates = 0;
block.SampleTimes = [C.Ts 0];
block.SimStateCompliance = 'DisallowSimState';
block.RegBlockMethod('Start', @start);
block.RegBlockMethod('Outputs', @render);
end

function C = settings()
C.Ts = 0.05;                   % 20 graphics updates / simulation second
C.arm = 0.24;                  % center to rotor [m]
C.rotorRadius = 0.085;          % graphical rotor radius [m]
C.loadSize = [1.2 1.2 0.16];    % graphical box dimensions [m]
C.follow = true;               % true: camera bounds follow payload
C.halfSpan = [4 4 4];          % following view half-width [m]
C.fixedLimits = [-4 4; -4 4; -5 2]; % used when follow=false
C.trails = true;
C.maxTrailPoints = 2000;
C.view = [40 25];              % azimuth, elevation [deg]
end

function tag = figureTag(block)
tag = sprintf('Draw_UAV_Load_%.0f',block.BlockHandle);
end

function start(block)
    C = settings();
    tag = figureTag(block);
    old = findall(groot,'Type','figure','Tag',tag);
    if ~isempty(old), delete(old); end
    fig = figure( ...
    'Name',['UAV + payload: ' getfullname(block.BlockHandle)],...
    'NumberTitle','off',...
    'Color','k',...
    'Tag',tag);
    ax = axes('Parent',fig);
    hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    set(ax,'ZDir','reverse','Projection','perspective');
    xlabel(ax,'X [m]'); ylabel(ax,'Y [m]'); zlabel(ax,'Z down [m]');
    view(ax,C.view(1),C.view(2));
    xlim(ax,C.fixedLimits(1,:)); ylim(ax,C.fixedLimits(2,:));
    zlim(ax,C.fixedLimits(3,:));
    
    H.ax = ax; H.C = C; H.lastTime = -inf;
    H.drone = gobjects(4,1); H.cable = gobjects(4,1);
    H.label = gobjects(4,1); H.trail = gobjects(5,1);
    colors = lines(4);
    for i = 1:4
        tr = hgtransform('Parent',ax);
        H.drone(i) = tr;
        a = C.arm;
        % Two arms and four rotor rings, all defined in the body frame.
        line('Parent',tr,'XData',[-a a],'YData',[0 0],'ZData',[0 0],...
            'Color',colors(i,:),'LineWidth',3);
        line('Parent',tr,'XData',[0 0],'YData',[-a a],'ZData',[0 0],...
            'Color',colors(i,:),'LineWidth',3);
        centers = [a 0 -a 0; 0 a 0 -a];
        theta = linspace(0,2*pi,32);
        for j = 1:4
            line('Parent',tr,...
                'XData',centers(1,j)+C.rotorRadius*cos(theta),...
                'YData',centers(2,j)+C.rotorRadius*sin(theta),...
                'ZData',zeros(size(theta)),...
                'Color',colors(i,:),'LineWidth',1.4);
        end
        % Red +body-x arm indicates heading.
        line('Parent',tr,'XData',[0 a],'YData',[0 0],'ZData',[0 0],...
            'Color',[0.85 0.1 0.1],'LineWidth',4);
        H.cable(i) = plot3(ax,nan,nan,nan,'-','Color',colors(i,:),...
            'LineWidth',1.6);
        H.label(i) = text(ax,0,0,0,sprintf('D%d',i),...
            'Color',colors(i,:),'FontWeight','bold');
        H.trail(i) = animatedline('Parent',ax,'Color',colors(i,:),...
            'LineStyle',':','MaximumNumPoints',C.maxTrailPoints);
    end
    H.load = hgtransform('Parent',ax);
    d = C.loadSize/2;
    vertices = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;...
                -1 -1 1;1 -1 1;1 1 1;-1 1 1].*d;
    faces = [1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8];
    patch('Parent',H.load,'Vertices',vertices,'Faces',faces,...
        'FaceColor',[0.85 0.65 0.25],'FaceAlpha',0.65,...
        'EdgeColor',[0.25 0.20 0.10]);
    H.attach = plot3(ax,nan,nan,nan,'ko','MarkerFaceColor','k','MarkerSize',4);
    H.trail(5) = animatedline('Parent',ax,'Color',[0.15 0.15 0.15],...
        'LineWidth',1.4,'MaximumNumPoints',C.maxTrailPoints);
    H.title = title(ax,'Waiting for simulation states...');
    fig.UserData = H;
    drawnow;
end

function render(block)
fig = findall(groot,'Type','figure','Tag',figureTag(block));
if isempty(fig), return; end
fig = fig(1);
H = fig.UserData;
if isempty(H) || ~isgraphics(H.ax), return; end
t = block.CurrentTime;
if t <= H.lastTime, return; end % avoid repeated draws at the same hit
pD = reshape(block.InputPort(1).Data,3,4);
phiD = reshape(block.InputPort(2).Data,3,4);
xL = block.InputPort(3).Data;
r = reshape(block.InputPort(4).Data,3,4);
if any(~isfinite([pD(:);phiD(:);xL(:);r(:)]))
    return; % solver/model diagnostics remain responsible for bad states
end
pL = xL(1:3);
RL = rotationZYX(xL(7:9));
attachments = pL + RL*r;
H.load.Matrix = [RL pL;0 0 0 1];
for i = 1:4
    H.drone(i).Matrix = [rotationZYX(phiD(:,i)) pD(:,i);0 0 0 1];
    set(H.cable(i),'XData',[pD(1,i),attachments(1,i)],...
        'YData',[pD(2,i),attachments(2,i)],...
        'ZData',[pD(3,i),attachments(3,i)]);
    H.label(i).Position = (pD(:,i)+[0;0;-0.16]).';
    if H.C.trails
        addpoints(H.trail(i),pD(1,i),pD(2,i),pD(3,i));
    end
end
set(H.attach,'XData',attachments(1,:),'YData',attachments(2,:),...
    'ZData',attachments(3,:));
if H.C.trails, addpoints(H.trail(5),pL(1),pL(2),pL(3)); end
if H.C.follow
    h = H.C.halfSpan;
    xlim(H.ax,pL(1)+[-h(1),h(1)]);
    ylim(H.ax,pL(2)+[-h(2),h(2)]);
    zlim(H.ax,pL(3)+[-h(3),h(3)]);
end
H.title.String = sprintf('t = %.2f s | Load: [%.2f, %.2f, %.2f] m',...
    t,pL(1),pL(2),pL(3));
H.lastTime = t;
fig.UserData = H;
drawnow limitrate;
end

function R = rotationZYX(phi)
cr=cos(phi(1)); sr=sin(phi(1));
ct=cos(phi(2)); st=sin(phi(2));
cy=cos(phi(3)); sy=sin(phi(3));
R = [cy*ct,cy*st*sr-sy*cr,cy*st*cr+sy*sr;...
     sy*ct,sy*st*sr+cy*cr,sy*st*cr-cy*sr;...
     -st,ct*sr,ct*cr];
end

function RobotTest

TankRadius = 3.2;
TankDepth = 2.3;
NumRobots = 1;
RobotRad = 1/6;

HS.Fig = figure;
SS = get(0,'ScreenSize');
set(HS.Fig,'units','normalized','Position',[.1 .1 .8*SS(4)/SS(3) .8],'Name','Robot Test Bed','NumberTitle','off','Color',[.9 .9 .9],'menubar','none','windowstyle','normal','currentcharacter','a','closerequestfcn','set(gcf,''currentcharacter'',char(27));','WindowScrollWheelFcn',@ScrollZ);
HS.XY = axes('units','normalized','position',[.025 .45 .45 .45],'color','k','XTick',[],'YTick',[],'ZTick',[]);
thetas = (0:pi/30:2*pi)';
r = TankRadius;
X = r*cos(thetas);
Y = r*sin(thetas);
a = patch(r*cos(thetas),r*sin(thetas),[.2 .2 .8]);
set(a,'edgecolor','none','buttondownfcn',@AdjustXY);
r = 1.07*TankRadius;
X = [X; r*cos(thetas(end:-1:1))];
Y = [Y; r*sin(thetas(end:-1:1))];
a = patch(X,Y,[.7 .7 .7]);
set(a,'edgecolor','none');
HS.RobXY = zeros(1,NumRobots);
HS.AimPtsXY = zeros(1,NumRobots);
thetas = (0:pi/8:2*pi);
for i = 1:1:NumRobots
    HS.RobXY(i) = patch(RobotRad*cos(thetas),RobotRad*sin(thetas),'r');
    set(HS.RobXY(i),'edgecolor','none','buttondownfcn',@AdjustXY);
    HS.AimPtsXY(i) = patch(RobotRad*[1 1 .2 .2 -.2 -.2 -1 -1 -.2 -.2 .2 .2],RobotRad*[-.2 .2 .2 1 1 .2 .2 -.2 -.2 -1 -1 -.2],'g');
    set(HS.AimPtsXY(i),'edgecolor','none','buttondownfcn',@AdjustXY);
end

axis(10/9*TankRadius*[-1 1 -1 1]);

HS.Z = axes('units','normalized','position',[.525 .45 .2 .45],'color','k','XTick',[],'YTick',[],'ZTick',[]);
HS.RobZ = zeros(1,NumRobots);
HS.AimPtsZ = zeros(1,NumRobots);
a = patch(TankDepth*.75*[-1 1 1 -1]/2.25,TankDepth*[1 1 1.07 1.07],[.7 .7 .7]);
set(a,'edgecolor','none');
a = patch(TankDepth*.75*[-1 1 1 -1]/2.25,TankDepth*[0 0 1 1],[.2 .2 .8]);
set(a,'edgecolor','none','buttondownfcn',@AdjustZ);
for i = 1:1:NumRobots
    HS.RobZ(i) = patch(RobotRad*cos(thetas),RobotRad*sin(thetas)+TankDepth-RobotRad,'r');
    set(HS.RobZ(i),'edgecolor','none','buttondownfcn',@AdjustZ);
    HS.AimPtsZ(i) = patch(RobotRad*[1 1 .2 .2 -.2 -.2 -1 -1 -.2 -.2 .2 .2],RobotRad*[-.2 .2 .2 1 1 .2 .2 -.2 -.2 -1 -1 -.2]+TankDepth-RobotRad,'g');
    set(HS.AimPtsZ(i),'edgecolor','none','buttondownfcn',@AdjustZ);
end
axis(TankDepth*[-.75/2.25 .75/2.25 0 1.25]);

uicontrol('style','text','units','normalized','position',[.025 .91 .45 .04],'string','X/Y Position','Fontweight','Bold','FontSize',18,'backgroundcolor',[.9 .9 .9]);
uicontrol('style','text','units','normalized','position',[.525 .91 .2 .04],'string','Z Position','Fontweight','Bold','FontSize',18,'backgroundcolor',[.9 .9 .9]);
HS.Targets = zeros(NumRobots,3);
HS.Targets(:,3) = TankDepth-RobotRad;
HS.RobLocs = zeros(NumRobots,3);
HS.RobLocs(:,3) = TankDepth-RobotRad;
HS.TankDepth = TankDepth;
HS.RobotRad = RobotRad;
guidata(gcf,HS);

while double(get(HS.Fig,'currentcharacter')) ~= 27
    pause(.2);
    HS = guidata(HS.Fig);
    for i = 1:1:NumRobots
        URLString = ['http://pod.princeton.edu/beluga/beluga.php?robot=' sprintf('%i',i-1) '&go_x=' sprintf('%1.2f',HS.Targets(i,1)) '&go_y=' sprintf('%1.2f',HS.Targets(i,2)) '&go_z=' sprintf('%1.2f',HS.Targets(i,3))];
        out = str2num(urlread(URLString));
        %    out = urlread('http://pod.princeton.edu/beluga/beluga.php?robot=0&go_x=1.2&go_y=-2.1&go_z=0');
%         out = .8*HS.RobLocs(i,:) + .2*HS.Targets(i,:);
        HS.RobLocs(i,:) = out(4*(i-1)+2:4*(i-1)+4);
        set(HS.RobXY(i),'XData',RobotRad*cos(thetas)+HS.RobLocs(i,1),'YData',RobotRad*sin(thetas)+HS.RobLocs(i,2));
        set(HS.RobZ(i),'YData',RobotRad*sin(thetas)+HS.RobLocs(i,3));
    end
    guidata(HS.Fig,HS);
end
delete(HS.Fig);

function AdjustXY(src,event)
HS = guidata(gcf);
CP = get(HS.XY,'CurrentPoint');
CP = CP(1,1:2);
HS.Targets(1,1:2) = CP;
set(HS.AimPtsXY(1),'XData',HS.RobotRad*[1 1 .2 .2 -.2 -.2 -1 -1 -.2 -.2 .2 .2]+HS.Targets(1,1),'YData',HS.RobotRad*[-.2 .2 .2 1 1 .2 .2 -.2 -.2 -1 -1 -.2]+HS.Targets(1,2));
guidata(gcf,HS);

function AdjustZ(src,event)
HS = guidata(gcf);
CP = get(HS.Z,'CurrentPoint');
CP = CP(1,2);
HS.Targets(1,3) = CP;
HS.Targets(1,3) = max([min([HS.Targets(1,3) HS.TankDepth-HS.RobotRad]) HS.RobotRad]);
set(HS.AimPtsZ(1),'XData',HS.RobotRad*[1 1 .2 .2 -.2 -.2 -1 -1 -.2 -.2 .2 .2],'YData',HS.RobotRad*[-.2 .2 .2 1 1 .2 .2 -.2 -.2 -1 -1 -.2]+HS.Targets(1,3));
guidata(gcf,HS);

function ScrollZ(src,event)
HS = guidata(gcf);
HS.Targets(1,3) = HS.Targets(1,3)+event.VerticalScrollCount*event.VerticalScrollAmount*-.025;
HS.Targets(1,3) = max([min([HS.Targets(1,3) HS.TankDepth-HS.RobotRad]) HS.RobotRad]);
set(HS.AimPtsZ(1),'XData',HS.RobotRad*[1 1 .2 .2 -.2 -.2 -1 -1 -.2 -.2 .2 .2],'YData',HS.RobotRad*[-.2 .2 .2 1 1 .2 .2 -.2 -.2 -1 -1 -.2]+HS.Targets(1,3));
guidata(gcf,HS);





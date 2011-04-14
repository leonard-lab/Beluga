function varargout = MonitorGUI(varargin)
% MONITORGUI M-file for MonitorGUI.fig
%      MONITORGUI, by itself, creates a new MONITORGUI or raises the existing
%      singleton*.
%
%      H = MONITORGUI returns the handle to a new MONITORGUI or the handle to
%      the existing singleton*.
%
%      MONITORGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MONITORGUI.M with the given input arguments.
%
%      MONITORGUI('Property','Value',...) creates a new MONITORGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MonitorGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MonitorGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MonitorGUI

% Last Modified by GUIDE v2.5 14-Apr-2011 10:14:59

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MonitorGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @MonitorGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before MonitorGUI is made visible.
function MonitorGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MonitorGUI (see VARARGIN)

% Choose default command line output for MonitorGUI
handles.output = hObject;

handles.paused = 1;

handles.timer = timer('timerFcn', {@timerFcn, handles.figure1}, 'BusyMode', 'Queue', 'ExecutionMode', 'FixedRate', 'Period', 0.5);

drawWireFrame(handles.axesMain);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MonitorGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function drawWireFrame(axes_handle)

r1_W = 0.24;   % radius of center piece at bottom (this is a guess!)
r2_W = 6.547/2;  % radius of tank (est, based on pro-e model)
r3_W = r2_W - .1;  % radius to inside of corners at the top (guess)
z3_W = 2.464;       % height to top of tank edge from bottom of inside of tank (est, based on pro-E model)
SA = [0 : 2*pi/14 : 2*pi*(1-1/14)]';

c1 = r2_W*exp(i*linspace(0, 2*pi));
o1 = [zeros(length(SA),1)];
b1 = r2_W*exp(i*SA);
z1 = [zeros(length(SA),2) z3_W*ones(length(SA),1)];
plot3(axes_handle, real(c1), imag(c1), zeros(size(c1)), 'k-', ...
    real(c1), imag(c1), z3_W*ones(size(c1)), 'k-',...
    real([o1 b1 b1])', imag([o1 b1 b1])', z1', 'r-');
axis equal

% --- Outputs from this function are returned to the command line.
function varargout = MonitorGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function [x,y,z,success] = getPositions()

BELUGA_PHP = 'http://localhost/~dan/beluga/beluga.php';

[s, success] = urlread(BELUGA_PHP);
if success,
    d = sscanf(s, '%f ');
    if(numel(d) == 0 || mod(numel(d), 4) ~= 0),
        success = 0;
        x = 0;
        y = 0;
        z = 0;
    else
        x = d([2 : 4 : end]);
        y = d([3 : 4 : end]);
        z = d([4 : 4 : end]);
    end
else
    x = 0;
    y = 0;
    z = 0;
end

function timerFcn(src,event,handles) 
handles = guidata(handles);

[x, y, z, success] = getPositions();

if(~success),
    return;
end

axes(handles.axesMain);
[az,el] = view(handles.axesMain);
drawWireFrame(handles.axesMain);

hold(handles.axesMain, 'on');
h = plot3(handles.axesMain, x, y, z, 'go');
set(h, 'LineWidth', 2, 'MarkerSize', 10);
view(handles.axesMain, [az, el]);

hold(handles.axesMain, 'off');
drawnow

guidata(handles.figure1, handles);

% --- Executes on button press in buttonGoPause.
function buttonGoPause_Callback(hObject, eventdata, handles)
% hObject    handle to buttonGoPause (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if(handles.paused),
    handles.paused = 0;
    set(hObject, 'String', 'Pause');
    start(handles.timer);
else
    handles.paused = 1;
    set(hObject, 'String', 'Go');
    stop(handles.timer);
end

guidata(hObject, handles)


% --- Executes on button press in buttonQuit.
function buttonQuit_Callback(hObject, eventdata, handles)
% hObject    handle to buttonQuit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

stop(handles.timer);
delete(handles.timer);

close gcf


% --- Executes when figure1 is resized.
function figure1_ResizeFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

fp = get(hObject, 'position');
w = fp(3)-10;
h = fp(4)-8;
set(handles.axesMain, 'position', [5 6 w h])

p1 = get(handles.buttonGoPause, 'position');
sx = p1(1);  sy = p1(2);  w = p1(3);  h = p1(4);
set(handles.buttonQuit, 'position', [fp(3)-w-sx sy w h]);

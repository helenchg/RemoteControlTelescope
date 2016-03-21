function varargout = ArduinoGui(varargin)
% ARDUINOGUI MATLAB code for ArduinoGui.fig
%      ARDUINOGUI, by itself, creates a new ARDUINOGUI or raises the existing
%      singleton*.
%
%      H = ARDUINOGUI returns the handle to a new ARDUINOGUI or the handle to
%      the existing singleton*.
%
%      ARDUINOGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ARDUINOGUI.M with the given input arguments.
%
%      ARDUINOGUI('Property','Value',...) creates a new ARDUINOGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ArduinoGui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ArduinoGui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ArduinoGui

% Last Modified by GUIDE v2.5 20-Mar-2016 01:28:43

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ArduinoGui_OpeningFcn, ...
                   'gui_OutputFcn',  @ArduinoGui_OutputFcn, ...
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


% --- Executes just before ArduinoGui is made visible.
function ArduinoGui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ArduinoGui (see VARARGIN)

% Choose default command line output for ArduinoGui
handles.output = hObject;
clear a;
global a;
handles.user.stop = 0;
handles.user.a = 0;

% Update handles structure
guidata(hObject, handles);




% UIWAIT makes ArduinoGui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ArduinoGui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function comPort_Callback(hObject, eventdata, handles)
% hObject    handle to comPort (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of comPort as text
%        str2double(get(hObject,'String')) returns contents of comPort as a double


% --- Executes during object creation, after setting all properties.
function comPort_CreateFcn(hObject, eventdata, handles)
% hObject    handle to comPort (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_connect.
function pushbutton_connect_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_connect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% value = str2double(get(handles.comPort, 'String'));
com = sprintf('COM%s', handles.comPort.String);
delete(instrfind({'Port'},{com}));
handles.user.a = arduino(com, 'Mega2560');

% while(1)
%     value = a.readVoltage('A0');
%     set(handles.text_temp, 'String', value);
% end
guidata(hObject, handles);


% --- Executes on button press in pushbutton_disconnect.
function pushbutton_disconnect_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_disconnect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% open_ports = instrfind('Type', 'serial', 'Status', 'open');
% if ~isempty(open_ports)
%     fclose(open_ports);
% end
% set(handles.user.stop, 'Value', 1)
global a
while true
    set(handles.text_temp, 'String', get(a.readVoltage('A0')));
end
guidata(hObject, handles);

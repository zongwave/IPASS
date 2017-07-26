%   image_denoise.m - image denoise using wavelet
%
%   Licensed under the Apache License, Version 2.0 (the "License");
%   you may not use this file except in compliance with the License.
%   You may obtain a copy of the License at
%
%        http://www.apache.org/licenses/LICENSE-2.0
%
%   Unless required by applicable law or agreed to in writing, software
%   distributed under the License is distributed on an "AS IS" BASIS,
%   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%   See the License for the specific language governing permissions and
%   limitations under the License.
%
%   Author: Zong Wei <wei.zong@intel.com>
%

function varargout = image_denoise(varargin)
% IMAGE_DENOISE MATLAB code for image_denoise.fig
%      IMAGE_DENOISE, by itself, creates a new IMAGE_DENOISE or raises the existing
%      singleton*.
%
%      H = IMAGE_DENOISE returns the handle to a new IMAGE_DENOISE or the handle to
%      the existing singleton*.
%
%      IMAGE_DENOISE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in IMAGE_DENOISE.M with the given input arguments.
%
%      IMAGE_DENOISE('Property','Value',...) creates a new IMAGE_DENOISE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before image_denoise_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to image_denoise_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help image_denoise

% Last Modified by GUIDE v2.5 19-Jul-2017 11:27:11

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @image_denoise_OpeningFcn, ...
                   'gui_OutputFcn',  @image_denoise_OutputFcn, ...
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


% --- Executes just before image_denoise is made visible.
function image_denoise_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to image_denoise (see VARARGIN)

% Choose default command line output for image_denoise
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes image_denoise wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = image_denoise_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function set_param_Callback(hObject, eventdata, handles)
% hObject    handle to set_param (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
thresh = get( handles.set_param, 'value');
if rem(thresh, 1) ~= 0
    thresh = round(thresh);
    set(hObject,'Value', thresh)
end
set(handles.params,'string', thresh);

% --- Executes during object creation, after setting all properties.
function set_param_CreateFcn(hObject, eventdata, handles)
% hObject    handle to set_param (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject,'Value', 1);

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function params_Callback(hObject, eventdata, handles)
% hObject    handle to params (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of params as text
%        str2double(get(hObject,'String')) returns contents of params as a double


% --- Executes during object creation, after setting all properties.
function params_CreateFcn(hObject, eventdata, handles)
% hObject    handle to params (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
set(hObject,'Value', 1);


% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in wavelet_basis.
function wavelet_basis_Callback(hObject, eventdata, handles)
% hObject    handle to wavelet_basis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns wavelet_basis contents as cell array
%        contents{get(hObject,'Value')} returns selected item from wavelet_basis


% --- Executes during object creation, after setting all properties.
function wavelet_basis_CreateFcn(hObject, eventdata, handles)
% hObject    handle to wavelet_basis (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in thresh_setting.
function thresh_setting_Callback(hObject, eventdata, handles)
% hObject    handle to thresh_setting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns thresh_setting contents as cell array
%        contents{get(hObject,'Value')} returns selected item from thresh_setting


% --- Executes during object creation, after setting all properties.
function thresh_setting_CreateFcn(hObject, eventdata, handles)
% hObject    handle to thresh_setting (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in denoise.
function denoise_Callback(hObject, eventdata, handles)
% hObject    handle to denoise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
basis_index = get(handles.wavelet_basis,'value');
switch basis_index
    case 1
        wname = 'haar';
    case 2
        wname = 'db2';
    case 3
        wname = 'db4';
    case 4
        wname = 'db8';
    case 5
        wname = 'coif1';
    case 6
        wname = 'coif2';
    case 7
        wname = 'coif4';
    case 8
        wname = 'sym2';
    case 9
        wname = 'sym4';
    case 10
        wname = 'sym8';
    case 11
        wname = 'bior1.1';
    case 12
        wname = 'bior3.5';
    otherwise
        wname = 'haar';
end

thresh_index = get(handles.thresh_setting,'value');
switch thresh_index
    case 1
        tname = 'penalhi';
    case 2
        tname = 'penalme';
    case 3
        tname = 'penallo';
    case 4
        tname = 'sqtwolog';
    case 5
        tname = 'sqrtbal_sn';
    otherwise
        tname = 'penalhi';
end

tuning = get( handles.set_param, 'value');

wavelet_denoise(wname, tname, tuning);

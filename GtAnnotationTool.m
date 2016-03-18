function varargout = GtAnnotationTool(varargin)
% GTANNOTATIONTOOL MATLAB code for GtAnnotationTool.fig
%      GTANNOTATIONTOOL, by itself, creates a new GTANNOTATIONTOOL or raises the existing
%      singleton*.
%
%      H = GTANNOTATIONTOOL returns the handle to a new GTANNOTATIONTOOL or the handle to
%      the existing singleton*.
%
%      GTANNOTATIONTOOL('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GTANNOTATIONTOOL.M with the given input arguments.
%
%      GTANNOTATIONTOOL('Property','Value',...) creates a new GTANNOTATIONTOOL or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GtAnnotationTool_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GtAnnotationTool_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GtAnnotationTool

% Last Modified by GUIDE v2.5 30-Oct-2015 16:11:20

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @GtAnnotationTool_OpeningFcn, ...
    'gui_OutputFcn',  @GtAnnotationTool_OutputFcn, ...
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


% --- Executes just before GtAnnotationTool is made visible.
function GtAnnotationTool_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GtAnnotationTool (see VARARGIN)

% Choose default command line output for GtAnnotationTool
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

h = get(gcf,'Position');
h = [5, 200, h(3), h(4)];
set(gcf,'Position',h);

% UIWAIT makes GtAnnotationTool wait for user response (see UIRESUME)
%  uiwait(handles.figure_main);



% --- Outputs from this function are returned to the command line.
function varargout = GtAnnotationTool_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in radiobutton_lane.
function radiobutton_lane_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton_lane (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton_lane


% --- Executes on button press in pushbutton_show.
function pushbutton_show_Callback(hObject, eventdata, handles)
global flagShowAll;
flagShowAll = ~flagShowAll;


% --- Executes on button press in pushbutton_output.
function pushbutton_output_Callback(hObject, eventdata, handles)
global folderName;

global gt_lanes;
global gt_circles;
global gt_founts;
global gt_botts;
global gt_block;


%���ñ�����ļ���·�������ڶ�ȡ���ļ����¼��ϡ�CUHK_GT���ļ���
index_dir=strfind(folderName,filesep);
savePath = folderName(1:index_dir(end-1)-1);
seqFolder = folderName(index_dir(end)+1 : end);
saveFolder = fullfile(savePath,'CUHK_GT',seqFolder);

%���޴��ļ��е�����£������ƶ�·���µ��ļ���
if ~exist(saveFolder,'dir')
    mkdir(saveFolder);
end

%����lane,circle��6�����ͱ����ļ�
if ~isempty(gt_lanes)
    filePath = fullfile(saveFolder,'gt_lanes.mat');
    save(filePath,'gt_lanes');
end
if ~isempty(gt_circles)
    filePath = fullfile(saveFolder,'gt_circles.mat');
    save(filePath,'gt_circles');
end
if ~isempty(gt_founts)
    filePath = fullfile(saveFolder,'gt_founts.mat');
    save(filePath,'gt_founts');
end
if ~isempty(gt_botts)
    filePath = fullfile(saveFolder,'gt_botts.mat');
    save(filePath,'gt_botts');
end
if ~isempty(gt_block)
    filePath = fullfile(saveFolder,'gt_block.mat');
    save(filePath,'gt_block');
end


% --- Executes on button press in pushbutton_save.
function pushbutton_save_Callback(hObject, eventdata, handles)
%������Ӧ�Ľṹ��
global gt_lanes;
global gt_circles;
global gt_founts;
global gt_botts;
global gt_block;

global pts;
global maskPolygon;

[m,n] = size(pts);

flagLane = get(handles.radiobutton_lane,'value');
flagCArch = get(handles.radiobutton_carch,'value');
flagCCArch = get(handles.radiobutton_ccarch,'value');
flagFounts = get(handles.radiobutton_fountainhead,'value');
flagBotts = get(handles.radiobutton_bottleneck,'value');
flagBlock = get(handles.radiobutton_blocking,'value');


%���ݲ�ͬ��ѡ�����͸���Ӧ�Ľṹ�帳ֵ��������vertex���߶�maskPolygon
if flagLane
    if m>2
        gt_lanes{length(gt_lanes)+1}.path = maskPolygon;%������εľ���ֵ��path
        gt_lanes{length(gt_lanes)}.vertex = pts;%������ĵ㱣����vertex��
    end
    
elseif flagCArch
    if m>2
        gt_circles{length(gt_circles)+1}.path = maskPolygon;
        gt_circles{length(gt_circles)}.vertex = pts;
        gt_circles{length(gt_circles)}.direction = 1;
    end
    
elseif flagCCArch
    if m>2
        gt_circles{length(gt_circles)+1}.path = maskPolygon;
        gt_circles{length(gt_circles)}.vertex = pts;
        gt_circles{length(gt_circles)}.direction = 0;
    end
    
elseif flagFounts
    if m>2
        gt_founts{length(gt_founts)+1}.path = maskPolygon;
        gt_founts{length(gt_founts)}.vertex = pts;
    elseif m == 1
        gt_founts{length(gt_founts)+1}.point = pts;
    end
    
elseif flagBotts
    if m>2
        gt_botts{length(gt_botts)+1}.path = maskPolygon;
        gt_botts{length(gt_botts)}.vertex = pts;
    elseif m == 1
        gt_botts{length(gt_botts)+1}.point = pts;
    end
    
elseif flagBlock
    if m>2
        gt_block{length(gt_block)+1}.path = maskPolygon;
        gt_block{length(gt_block)}.vertex = pts;
    elseif m == 1
        gt_block{length(gt_block)+1}.point = pts;
    end
end
pts = [];
maskPolygon = [];
set(handles.figureWin,'visible','on');


% --- Executes on button press in pushbutton_clear.
function pushbutton_clear_Callback(hObject, eventdata, handles)%�����е�ȫ����ֵΪ��
global gt_lanes; gt_lanes = {};
global gt_circles; gt_circles = {};
global gt_founts; gt_founts = {};
global gt_botts; gt_botts = {};
global gt_block; gt_block = {};

global pts; pts = [];
global maskPolygon; maskPolygon = [];

global flagPolygon; flagPolygon = false;
global flagPoint; flagPoint = false;

set(handles.figureWin,'visible','on');


% % --- Executes on mouse press over axes background.
% function axes_figure_ButtonDownFcn(hObject, eventdata, handles)
% % hObject    handle to axes_figure (see GCBO)
% % eventdata  reserved - to be defined in a future version of MATLAB
% % handles    structure with handles and user data (see GUIDATA)
% 

% --- Executes on button press in pushbutton_load.
function pushbutton_load_Callback(hObject, eventdata, handles)
global flagStop;
flagStop = false;

global pts;
pts = [];

global currentPts;
currentPts = [];

global flagPolygon;
flagPolygon = false;
global flagPoint;
flagPoint = false;

global flagShowAll;
flagShowAll = false;

global gt_lanes; gt_lanes = {};
global gt_circles; gt_circles = {};
global gt_founts; gt_founts = {};
global gt_botts; gt_botts = {};
global gt_block; gt_block = {};

global im;

global maskPolygon;

global folderName;
folderName = uigetdir('H:\test','select the folder');%����Ĭ�ϵ�����ļ��д���,���õ����ն�ȡ���ļ���λ��
if isdir(folderName)
    set(handles.edit_path,'string',folderName);
    %     axes(handles.axes_figure);
    %     hfig = figure(1);
  handles.figureWin = figure(1);%����һ��ͼ��tagΪfigureWin
    guidata(hObject, handles);
    
    set(handles.figureWin,'WindowButtonUpFcn',@btn_up);%figureWin�еĵ��������Ӧ�ĺ���Ϊbtn_up
    set(handles.figureWin, 'WindowButtonMotionFcn',@btn_motion);%figureWin�е�����ƶ�������Ӧ�ĺ���Ϊbtn_motion
    
    fileList = dir(folderName);
    while 1
        if flagStop
            break;
        end
        
        for i = 1:1:length(fileList)
            if flagStop
                break;
            end
            
            imageName = fileList(i).name;
            
            if (~strcmp(imageName, '.') && ~strcmp(imageName, '..'))
                [~,name] = fileparts(imageName);
                imageFilePath = fullfile(folderName,[name,'.jpg']);
                if (exist(imageFilePath, 'file'))
                    set(handles.text_framenum,'string',name);
                    im = imread(imageFilePath);
                    
                    [imHigh,imWidth,~] = size(im);
                    strResolution = sprintf('%d x %d',imWidth,imHigh);
                    set(handles.text_resolution,'string',strResolution);%��ʾ��ȡͼƬ�ķֱ���
                    %     imshow(im,'InitialMagnification',100);
                    %     hold on;
                    
                    showPts(pts,flagPolygon,flagPoint,im,maskPolygon,flagShowAll,handles);%���ú������ϻ���
                    
                    h = get(handles.figureWin,'Position');
                    h = [600, 100, h(3), h(4)];
                    set(handles.figureWin,'Position',h);
                    
                    pause(0.01);
                end
            end
        end
    end
end



function edit_path_Callback(hObject, eventdata, handles)
% hObject    handle to edit_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit_path as text
%        str2double(get(hObject,'String')) returns contents of edit_path as a double


% --- Executes during object creation, after setting all properties.
function edit_path_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_path (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbutton_load.
function pushbutton_load_ButtonDownFcn(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function pushbutton_load_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pushbutton_load (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% --- Executes on button press in pushbutton_play.
function pushbutton_play_Callback(hObject, eventdata, handles)%play��ť����Ӧ��load��ť����Ӧ���ƣ��ڲ��ϲ�������֡������»�����ȥ��֮ǰ��������е�
global flagStop;
flagStop = false;%��ʼ��������֡

global pts;
%pts = [];

global currentPts;
currentPts = [];

global flagPolygon;
flagPolygon = false;
global flagPoint;
flagPoint = false;

global flagShowAll;
flagShowAll = false;

global im;

global maskPolygon;

global folderName
if isdir(folderName)
    set(handles.edit_path,'string',folderName);%��ʾ��ǰ��ȡͼ����ļ�������
    
    handles.figureWin = figure(1);
    guidata(hObject, handles);
    
    set(handles.figureWin,'WindowButtonUpFcn',@btn_up);
    set(handles.figureWin, 'WindowButtonMotionFcn',@btn_motion);
    
    fileList = dir(folderName);
    
    while 1
        if flagStop
            break;
        end
        
        for i = 1:1:length(fileList)
            if flagStop
                break;
            end
            
            imageName = fileList(i).name;
            
            if (~strcmp(imageName, '.') && ~strcmp(imageName, '..'))
                [~,name] = fileparts(imageName);
                imageFilePath = fullfile(folderName,[name,'.jpg']);
                if (exist(imageFilePath, 'file'))
                    set(handles.text_framenum,'string',name);
                    im = imread(imageFilePath);
                    
                    [imHigh,imWidth,~] = size(im);
                    strResolution = sprintf('%d x %d',imWidth,imHigh);
                    set(handles.text_resolution,'string',strResolution);
                    
                    showPts(pts,flagPolygon,flagPoint,im,maskPolygon,flagShowAll,handles);
                    
                    h = get(handles.figureWin,'Position');
                    h = [600, 100, h(3), h(4)];
                    set(handles.figureWin,'Position',h);
                    
                    pause(0.01);
                end
            end
        end
    end
end


% --- Executes on button press in pushbutton_stop.
function pushbutton_stop_Callback(hObject, eventdata, handles)
global flagStop;
flagStop = true;


% --- Executes during object creation, after setting all properties.
function axes_figure_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes_figure (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes_figure

function btn_up(h,evt)
global pts;
global flagPolygon;
global flagPoint;
global im;
global maskPolygon;

if flagPolygon
    temppt = get(gca,'CurrentPoint');%********************��ȡ��ǰ�������λ��*********
    [m,n] = size(pts);
    if m>1 && norm(pts(1,:) - temppt(1,1:2)) < 10%����ǰ�ص����һ�����ص�ľ������10��ʱ�������λ�ͼ���
        flagPolygon = false;%ֹͣ�˴λ������
        
        [mI,nI,~] = size(im);
        [X,Y] = meshgrid(1:nI,1:mI);
        maskPolygon = inpolygon(X,Y,pts(:,1),pts(:,2));%������㹹�ɵľ��ε��ڲ��ͱ��Ͼ���ֵΪ1�����渳ֵΪ0��
    else%�������ѡȡ�ö���ε���һ������
        pts = [pts; temppt(1,1:2)];%����ǰ�������ӵ�pts
    end
end

if flagPoint
    temppt = get(gca,'CurrentPoint');
    pts = temppt(1,1:2);%��pts����Ϊ��ǰ�����
    
    flagPoint = false;%�������㣬��ʱ���ͼ���޷�Ӧ
end


function btn_motion(h,evt)

function  showPts(pts,flagPolygon,flagPoint,im,maskPolygon,flagShowAll,handles)
global gt_lanes;
global gt_circles;
global gt_founts;
global gt_botts;
global gt_block;

gt_vertex = {};
gt_point = {};

% if flagShowAll
%     flagLane = get(handles.radiobutton_lane,'value');
%     flagCArch = get(handles.radiobutton_carch,'value');
%     flagCCArch = get(handles.radiobutton_ccarch,'value');
%     flagFounts = get(handles.radiobutton_fountainhead,'value');
%     flagBotts = get(handles.radiobutton_bottleneck,'value');
%     flagBlock = get(handles.radiobutton_blocking,'value');
%     if flagLane
%         gt_data = gt_lanes;
%     elseif flagCArch
%         gt_data = gt_circles;
%     elseif flagCCArch
%         gt_data = gt_circles;
%     elseif flagFounts
%         gt_data = gt_founts;
%     elseif flagBotts
%         gt_data = gt_botts;
%     elseif flagBlock
%         gt_data = gt_block;
%     end
%     
%     if ~isempty(gt_data)
%         for i = 1:length(gt_data)
%             dataNames = fieldnames(gt_data{i});
%             for j = 1:length(dataNames)
%                 if strcmp(dataNames{j},'path')
%                     im(:,:,2) = im(:,:,2) + 40 * uint8(gt_data{i}.path);
%                 elseif strcmp(dataNames{j},'vertex')
%                     gt_vertex{length(gt_vertex)+1} = gt_data{i}.vertex;
%                 elseif strcmp(dataNames{j},'point')
%                     gt_point{length(gt_point)+1} = gt_data{i}.point;
%                 end
%             end
%         end
%     end
% end

currentPts = get(gca,'CurrentPoint');%********************��ȡ��ǰ������ڵ�λ�ã��������������λ��*********

[m,n] = size(pts);%�����ĸ���m,nΪ2
if m>0
    if flagPolygon%������������Ӧm==2�������������
        %           currentPts = get(gca,'CurrentPoint');
        %           tempPts = [pts; currentPts(1,1:2)];
        imshow(im,'InitialMagnification',100);
        hold on;
        
        tempPts = [pts; currentPts(1,1:2)];%���λ�����ǰ�㣬��ǰ���������ƶ�
        plot(tempPts(:,1),tempPts(:,2),'--rs','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g', 'MarkerSize',10);%�����е㶼��plot��������ԭʼͼ����
        %           hold off;
    elseif m>2%��ʱ��ʾ������Ѿ����꣬��Ҫ��ʾ���������
        % fill(pts(:,1),pts(:,2),'b');
        im(:,:,2) = im(:,:,2) + 40 * uint8(maskPolygon);%����ѡ�Ķ����������ͼƬ����ʾ����
        tempPts = [pts; pts(1,:)];%���һ�������һ�����غ�
        imshow(im,'InitialMagnification',100);
        hold on
        plot(tempPts(:,1),tempPts(:,2),'--rs','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g', 'MarkerSize',10);
        %           hold off;
    elseif m == 1%����
        imshow(im,'InitialMagnification',100);
        hold on;
        plot(pts(1,1),pts(1,2),'--rs','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g', 'MarkerSize',10);%���ڸ���Ϊ1��ֻ����һ����
    end
    
else
    imshow(im,'InitialMagnification',100);
    hold on;
end

if flagPoint
    imshow(im,'InitialMagnification',100);
    hold on;
    
    plot(currentPts(:,1),currentPts(:,2),'--rs','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g', 'MarkerSize',10);%�������λ���ƶ�
end

% for i = 1: length(gt_vertex)
%     tempPts = [gt_vertex{i}; gt_vertex{i}(1,:)];
%     plot(tempPts(:,1),tempPts(:,2),'--rs','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g', 'MarkerSize',10);
% end
% for i = 1: length(gt_point)
%     plot(gt_point{i}(1,1),gt_point{i}(1,2),'rs','LineWidth',1,'MarkerEdgeColor','k','MarkerFaceColor','g', 'MarkerSize',10);
% end
hold off;

strCurrentPosition = sprintf('(%d, %d)',round(currentPts(1,1)),round(currentPts(1,2)));
set(handles.text_currentposition,'string',strCurrentPosition);
%   text(round(currentPts(1,1)),round(currentPts(1,2)),strCurrentPosition);



% --- Executes on button press in pushbutton_creatPolygon.
function pushbutton_creatPolygon_Callback(hObject, eventdata, handles)
global pts;
global flagPolygon;
global flagPoint;
global maskPolygon;
pts = [];
flagPolygon = true;
flagPoint = false;
maskPolygon = [];
set(handles.figureWin,'visible','on');



% --- Executes on button press in pushbutton_creatPoint.
function pushbutton_creatPoint_Callback(hObject, eventdata, handles)
global pts;
global flagPolygon;
global flagPoint;
global maskPolygon;
pts = [];
flagPolygon = false;
flagPoint = true;
maskPolygon = [];
set(handles.figureWin,'visible','on');


% --- Executes when user attempts to close figure_main.
function figure_main_CloseRequestFcn(hObject, eventdata, handles)
global flagStop;
flagStop = true;

if isfield(handles,'figureWin') && ishandle(handles.figureWin)
    close(handles.figureWin);
end

delete(hObject);

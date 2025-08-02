%please run seperately

%% part A
clear all;

% % TCP Host and Port settings
host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
%host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;

% Calling the constructor of rtde to setup tcp connction
rtde = rtde(host,port);

% Setting home
home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0];



% Creating a path array
path = [];

% setting move parameters
v = 0.5;
a = 1.2;
blend = 0.0005;
startup_rvc; % Startup the rvc toolbox
scale = 0.04;
load hershey;
str = '0123';
path_str = [];
pre_i = 1;
offset = 0;
pre_traj_x = -588.53;
for i = 1:length(str)
    char = str(i)
    character = hershey{char};
    path = [scale*character.stroke; zeros(1,numcols(character.stroke))] % create the path 
   

% Where ever there is an nan it indicates that we need to lift up.
    k = find(isnan(path(1,:)))

% At these positions add in a z hight
    path(:,k) = path(:,k-1); path(3,k) = 0.8*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
    traj = [path'*1000] % convert to the mm units so that w35e can use the rtde toolbox
    % Generate a plot of what we are expecting
   
    scatter3(traj(:,1), traj(:,2), traj(:,3));
    hold on
    plot3(traj(:,1), traj(:,2), traj(:,3));
    
    % Creating a paths array
   
    for j = 1:length(path)
        disp(j);
        point = [[(traj(j,1:3) + [pre_traj_x, -350 35]),(home(4:6))],a,v,0,blend]
        if j ==1
            paths = point
        else
            paths = cat(1,paths,point)
        end
    end

    point(:,3) = point(:,3)+15
    paths = cat(1,paths,point)
    pre_traj_x = max(paths(:,1))
    %p_next_start = path(end,:);
    if isempty(path_str)
        path_str = paths
    else
        path_str = cat(1,path_str,paths)
    end
    
end



%%NOW USE THE RTDE TOOLBOX TO EXECUTE THIS PATH!




% Execute the movement!
poses = rtde.movej(home);
poses = rtde.movej(path_str);

rtde.drawPath(poses);

rtde.close;
disp('Program Complete')




%% Part B
clear all;

% % TCP Host and Port settings
host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
%host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;

% Calling the constructor of rtde to setup tcp connction
rtde = rtde(host,port);

% Setting home
home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0];



% Creating a path array
path = [];

% setting move parameters
v = 0.5;
a = 1.2;
blend = 0.0005;
startup_rvc; % Startup the rvc toolbox
scale = 0.04;
load hershey;
str = '123';
theta = -30;
x = 0;
y = 0;
%T = SE2(x,y,theta,'deg')
T = transl2(x,y)*trot2(theta,'deg')
path_str = [];
pre_i = 1;
offset = 0;
pre_traj_x = 0;
for i = 1:length(str)
    char = str(i)
    character = hershey{char};
    path = [scale*character.stroke; zeros(1,numcols(character.stroke))] % create the path 
   

% Where ever there is an nan it indicates that we need to lift up.
    k = find(isnan(path(1,:)))

% At these positions add in a z hight
    path(:,k) = path(:,k-1); path(3,k) = 0.8*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
    
    traj = [path'*1000] % convert to the mm units so that w35e can use the rtde toolbox
   
    % Generate a plot of what we are expecting
   
    scatter3(traj(:,1), traj(:,2), traj(:,3));
    hold on
    plot3(traj(:,1), traj(:,2), traj(:,3));
    
    % Creating a paths array
   
    for j = 1:length(path)
        disp(j);
        point = [[(traj(j,1:3) + [pre_traj_x, 0 35]),(home(4:6))],a,v,0,blend]
        if j ==1
            paths = point
        else
            paths = cat(1,paths,point)
        end
    end

    point(:,3) = point(:,3)+15
    paths = cat(1,paths,point)
    pre_traj_x = max(paths(:,1))
    %p_next_start = path(end,:);
    if isempty(path_str)
        path_str = paths
    else
        path_str = cat(1,path_str,paths)
    end
    [rows, cols] = size(path_str);
    path_str = path_str'
    for u = 1:rows
        path_str(1:3,u) = T*path_str(1:3,u)
    end
    path_str = path_str'
end
path_str(:,1:3) = path_str(:,1:3)+[-588.53+x -350+y 0]


%%NOW USE THE RTDE TOOLBOX TO EXECUTE THIS PATH!




% Execute the movement!
poses = rtde.movej(home);
poses = rtde.movej(path_str);

rtde.drawPath(poses);

rtde.close;
disp('Program Complete')

%% part C
clear all;

% % TCP Host and Port settings
host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
%host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;

% Calling the constructor of rtde to setup tcp connction
rtde = rtde(host,port);

% Setting home
home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0];



% Creating a path array
path = [];

% setting move parameters
v = 0.5;
a = 1.2;
blend = 0.00005;

startup_rvc; % Startup the rvc toolbox
scale = 0.04;
load hershey;
str = '10*10=';
operators = '+-*';
pre_traj_x = -588.53;
pre_traj_y = -350;
flag_num1 = 1;
flag_num2 = 1;
flag_result = 1;
for n = 1:length(operators)
    symbol_index = strfind(str,operators(n))
    if ~isempty(symbol_index)
        break;
    end
end
num1 = str2num(str(1:symbol_index-1))
num2 = str2num(str(symbol_index+1:end-1))
switch str(symbol_index)
    case '+'
        result = num1+num2
    case "-"
        result = num1-num2
    case "*"
        result = num1*num2
end
str(end) = ''
str3 = str(symbol_index)

str = [str(1:symbol_index-1) str(symbol_index+1:end) str(symbol_index)];
result = num2str(result)
%str = strcat(str,result)
str1 = num2str(num1)
str2 = num2str(num2)
str4 = num2str(result)
path_str1 = []
path_str2 = []
path_str3 = []
path_str4 = []
%%num1
for i = 1:length(str1) %draw num1 first
    char = str1(i)
    character = hershey{char};
    path = [scale*character.stroke; zeros(1,numcols(character.stroke))] % create the path 
   

% Where ever there is an nan it indicates that we need to lift up.
    k = find(isnan(path(1,:)))

% At these positions add in a z hight
    path(:,k) = path(:,k-1); path(3,k) = 0.8*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
    traj = [path'*1000] % convert to the mm units so that we can use the rtde toolbox
% Generate a plot of what we are expecting
   
    scatter3(traj(:,1), traj(:,2), traj(:,3));
    hold on
    plot3(traj(:,1), traj(:,2), traj(:,3));
    
    % Creating a paths array
   
    length_path = length(path);
    for j = 1:length_path
        disp(j);
        point = [[(traj(j,1:3) + [pre_traj_x, -350 35]),(home(4:6))],a,v,0,blend]
        
        if j == 1
            paths1 = point
        else
            paths1 = cat(1,paths1,point)
        end

    end
    point(:,3) = point(:,3)+15
    paths1 = cat(1,paths1,point)
    pre_traj_x = max(paths1(:,1))
    if isempty(path_str1)
        path_str1 = paths1
    else
        path_str1 = cat(1,path_str1,paths1)
    end
end
pre_traj_x = -588.53;
pre_traj_y = min(paths1(:,2))
%%num2
for i = 1:length(str2) %draw num2
    char = str2(i)
    character = hershey{char};
    path = [scale*character.stroke; zeros(1,numcols(character.stroke))] % create the path 
   

% Where ever there is an nan it indicates that we need to lift up.
    k = find(isnan(path(1,:)))

% At these positions add in a z hight
    path(:,k) = path(:,k-1); path(3,k) = 0.8*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
    traj = [path'*1000] % convert to the mm units so that we can use the rtde toolbox
% Generate a plot of what we are expecting

    % Creating a paths array
   
    length_path = length(path);
    for j = 1:length_path
        disp(j);
        point = [[(traj(j,1:3) + [pre_traj_x, pre_traj_y-50*character.top 35]),(home(4:6))],a,v,0,blend]
        
        if j == 1
            paths2 = point
        else
            paths2 = cat(1,paths2,point)
        end

    end
    point(:,3) = point(:,3)+15
    paths2 = cat(1,paths2,point)
    pre_traj_x = max(paths2(:,1))
   
    if isempty(path_str2)
        path_str2 = paths2
    else
        path_str2 = cat(1,path_str2,paths2)
    end
end
pre_traj_y = min(paths2(:,2))
%%symbol
for i = 1:length(str3) %draw symbol
    char = str3(i)
    if char == '*'
        character = hershey{'x'};
    else
        character = hershey{char};
    end
    path = [scale*character.stroke; zeros(1,numcols(character.stroke))] % create the path 
   

% Where ever there is an nan it indicates that we need to lift up.
    k = find(isnan(path(1,:)))

% At these positions add in a z hight
    path(:,k) = path(:,k-1); path(3,k) = 0.8*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
    traj = [path'*1000] % convert to the mm units so that we can use the rtde toolbox
% Generate a plot of what we are expecting

    % Creating a paths array
   

    if str(symbol_index+1) == '-'
       length_path = length(path)-1;
    else
       length_path = length(path);
    end
    for j = 1:length_path
        disp(j);
        point = [[(traj(j,1:3) + [pre_traj_x, pre_traj_y-10*character.top 35]),(home(4:6))],a,v,0,blend]
        
        if j == 1
            paths3 = point
        else
            paths3 = cat(1,paths3,point)
        end

    end
    point(:,3) = point(:,3)+15
    paths3 = cat(1,paths3,point)
    pre_traj_x = max(paths3(:,1))
    pre_traj_y = min(paths3(:,2))
    if isempty(path_str3)
        path_str3 = paths3
    else
        path_str3 = cat(1,path_str3,paths3)
    end
end
pre_traj_x = -588.53;
%%num4
for i = 1:length(str4) %draw num4
    char = str4(i)
    character = hershey{char};
    path = [scale*character.stroke; zeros(1,numcols(character.stroke))] % create the path 
   

% Where ever there is an nan it indicates that we need to lift up.
    k = find(isnan(path(1,:)))

% At these positions add in a z hight
    path(:,k) = path(:,k-1); path(3,k) = 0.8*scale; % Determine the hight of the lift up motions. 0.2 * scale is the height. 0.2 is in m
    traj = [path'*1000] % convert to the mm units so that we can use the rtde toolbox
% Generate a plot of what we are expecting
   
    scatter3(traj(:,1), traj(:,2), traj(:,3));
    hold on
    plot3(traj(:,1), traj(:,2), traj(:,3));
    
    % Creating a paths array
   
    length_path = length(path);
    for j = 1:length_path
        disp(j);
        point = [[(traj(j,1:3) + [pre_traj_x, pre_traj_y-60*character.top 35]),(home(4:6))],a,v,0,blend]
        
        if j == 1
            paths4 = point
        else
            paths4 = cat(1,paths4,point)
        end

    end
    point(:,3) = point(:,3)+15
    paths4 = cat(1,paths4,point)
    pre_traj_x = max(paths4(:,1))
    if isempty(path_str4)
        path_str4 = paths4
    else
        path_str4 = cat(1,path_str4,paths4)
    end
end

%%NOW USE THE RTDE TOOLBOX TO EXECUTE THIS PATH!

path_str = cat(1,path_str1,path_str2,path_str3,path_str4)


% Execute the movement!
poses = rtde.movej(home);
poses = rtde.movej(path_str);

rtde.drawPath(poses);

rtde.close;
disp('Program Complete')
clear all;
clc;
v = 0.5;
a = 1.2;
blend = 0.0005;
home = [-588.53, -133.30, 227.00, 2.221, 2.221, 0];
path_move = [];
startup_rvc;
camList = webcamlist
%host = '127.0.0.1'; % THIS IP ADDRESS MUST BE USED FOR THE VIRTUAL BOX VM
%host = '192.168.230.128'; % THIS IP ADDRESS MUST BE USED FOR THE VMWARE
host = '192.168.0.100'; % THIS IP ADDRESS MUST BE USED FOR THE REAL ROBOT
port = 30003;
vacuumport = 63352;
rtde = rtde(host,port);
vacuum = vacuum(host,vacuumport);
rtde.movej(home+[0,0,190,0,0,0])
pause(1)
vacuum.grip()
pause(1)
vacuum.release()
cam = webcam(1);
pause(3);
real_img = snapshot(cam);
figure
imshow(real_img)


%vacuum = vacuum(host,vacuumport)
%% Part A
I = real_img;
%I = imread('3_ttest.jpg');
%I = imread('new.jpg');
%I = I+50;
I1 = I;
I2 = I;
I3 = I;
I4 = I;
[ids,locs,detectedFamily] = readArucoMarker(I,"DICT_4X4_250");
numMarkers = length(ids);
allcenter = [];
for i = 1:numMarkers
  loc = locs(:,:,i);

  % Display the marker ID and family
  disp("Detected marker ID, Family: " + ids(i) + ", " + detectedFamily(i))
  
  % Insert marker edges
  I = insertShape(I,"polygon",{loc},Opacity=1,ShapeColor="green",LineWidth=4);
 
  % Insert marker corners
  markerRadius = 6;
  numCorners = size(loc,1);
  markerPosition = [loc,repmat(markerRadius,numCorners,1)];
  I = insertShape(I,"FilledCircle",markerPosition,ShapeColor="red",Opacity=1);
   
  % Insert marker IDs
  center = mean(loc);
  I = insertText(I,center,ids(i),FontSize=30,BoxOpacity=1);
  allcenter = [allcenter;center,ids(i)];
end
sorted_all_center = sortrows(allcenter, 3);
figure
imshow(I)
% corner5 = sorted_all_center(sorted_all_center(:,3) == 5,1:2);
% corner6 = sorted_all_center(sorted_all_center(:,3) == 6,1:2);
% corner7 = sorted_all_center(sorted_all_center(:,3) == 7,1:2);
% corner8 = sorted_all_center(sorted_all_center(:,3) == 8,1:2);
% allcorner = [corner5;corner6;corner7;corner8]
% x = allcorner(:,1);
% y = allcorner(:,2);
% width = sqrt((x(1)-x(2))^2+(y(1)-y(2))^2)
% len = sqrt((x(1)-x(3))^2+(y(1)-y(3))^2)
% %destPoints = [0 width; 0 0; len width; len 0];
% destPoints = [len width; len 0; 0 width; 0 0];
% %destPoints = destPoints+50;
% trans = fitgeotrans(allcorner, destPoints, 'projective');
% xlimits = [-22 234];
% ylimits = [-23 163];
% outputview = imref2d(size(I3),xlimits,ylimits);
% warpedImg = imwarp(I3,trans,'OutputView',outputview);
%[x,map] = imwarp(I3,trans);

% figure
% imshow(warpedImg);
% figure
% ITTT = imwarp(warpedImg,invert(trans));
% imshow(ITTT)
% figure
% imshow(I3)

for i = 1:length(ids)
    % 获取ArUco标记的中心点
    corners_I = locs(:,:,i)
    centerX_marker_I = mean(corners_I(:, 1))
    centerY_marker_1 = mean(corners_I(:, 2))
end

plot(sorted_all_center(1,:),sorted_all_center(2,:),'r*')
hold on
imshow(I)
sorted_all_center = sorted_all_center';
img_coordinate = [sorted_all_center(1:2,1),sorted_all_center(1:2,2),sorted_all_center(1:2,3),sorted_all_center(1:2,4)] %the sequence can be seen from id1
img_coordinate_gameboard = [sorted_all_center(1:2,5),sorted_all_center(1:2,6),sorted_all_center(1:2,7),sorted_all_center(1:2,8)] %the sequence can be seen from id1
x_world = [-230 -230 -990 -990]; %1234
y_world = [60 -520 60 -520];
% x_world = [-990 -230 -990 -230]; %3142
% y_world = [60 60 -520 -520];
% x_world = [-230 -230 -990 -990]; %2143
% y_world = [-520 60 -520 60];

x_world_board3 = [-760 -610 -355  -200];
y_world_board3 = [-185 45 -460  -225];

world_coordinate_board3 = [x_world_board3;y_world_board3];
% x_world = [-990 -990 -230 -230]; %4321
% y_world = [-520 60 -520 60];
world_coordinate = [x_world;y_world];
tform = fitgeotform2d(img_coordinate', world_coordinate','projective');
tform_board3 = fitgeotrans(img_coordinate_gameboard', world_coordinate_board3','projective');
figure
%I3 = imwarp(I1,tform);
xlimits = [-1060 -150];
ylimits = [-600 150];
outputView = imref2d(size(I),xlimits,ylimits);
I2 = imwarp(I1,tform,'Outputview',outputView);
imshow(I2)
[I2_ids,I2_locs] = readArucoMarker(I3,"DICT_4X4_250");
I2_allcenter = [];
for i = 1:length(I2_ids)    
    I2_loc = I2_locs(:,:,i);
    I2_center = mean(I2_loc)
    I2_allcenter = [I2_allcenter;I2_center,I2_ids(i)];
end
I2_sorted_all_center = sortrows(I2_allcenter, 3);
I2_corner5 = I2_sorted_all_center(I2_sorted_all_center(:,3) == 5,1:2);
I2_corner6 = I2_sorted_all_center(I2_sorted_all_center(:,3) == 6,1:2);
I2_corner7 = I2_sorted_all_center(I2_sorted_all_center(:,3) == 7,1:2);
I2_corner8 = I2_sorted_all_center(I2_sorted_all_center(:,3) == 8,1:2);
allcorner = [I2_corner5;I2_corner6;I2_corner7;I2_corner8]
x = allcorner(:,1);
y = allcorner(:,2);
% width = sqrt((x(1)-x(2))^2+(y(1)-y(2))^2)+0.5*10*abs(I2_loc(1,1)-I2_loc(2,1))
% len = sqrt((x(1)-x(3))^2+(y(1)-y(3))^2)+0.5*10*abs(I2_loc(1,1)-I2_loc(2,1))
width = sqrt((x(1)-x(2))^2+(y(1)-y(2))^2)
len = sqrt((x(1)-x(3))^2+(y(1)-y(3))^2)
%destPoints = [0 width; 0 0; len width; len 0];
destPoints = [len width; len 0; 0 width; 0 0];
%destPoints = destPoints+50;
trans = fitgeotrans(allcorner, destPoints, 'projective');
xlimits = [-20 260];
ylimits = [-25 175];
% xlimits = [-30 245];
% ylimits = [-30 170];
outputView = imref2d(size(I),xlimits,ylimits);
warpedImg = imwarp(I3, trans,'OutputView',outputView);
[x,map] = imwarp(I3, trans);
figure
imshow(x,map)
hold on
% figure
% IT = imwarp(warpedImg,invert(trans));
% ITT = imwarp(IT,invert(tform))
% imshow(ITT)

% sorted_all_center = sortrows(allcenter, 3);
% sorted_all_center(:,1:2) = transformPointsForward(tform,sorted_all_center(:,1:2))
% loc = transformPointsForward(tform,loc)
% corner5 = sorted_all_center(sorted_all_center(:,3) == 5,1:2);
% corner6 = sorted_all_center(sorted_all_center(:,3) == 6,1:2);
% corner7 = sorted_all_center(sorted_all_center(:,3) == 7,1:2);
% corner8 = sorted_all_center(sorted_all_center(:,3) == 8,1:2);
% allcorner = [corner5;corner6;corner7;corner8]

% x = allcorner(:,1);
% y = allcorner(:,2);
% width = sqrt((x(1)-x(2))^2+(y(1)-y(2))^2)
% len = sqrt((x(1)-x(3))^2+(y(1)-y(3))^2)
% %destPoints = [0 width; 0 0; len width; len 0];
% destPoints = [len width; len 0; 0 width; 0 0];
% %destPoints = destPoints+50;
% trans = fitgeotrans(allcorner, destPoints, 'projective');
% I1 = imwarp(warpedImg,tform);
% figure
% imshow(I1)
% xlimits = [1490 2040];
% ylimits = [-260 110];
% outputView = imref2d(size(I3),xlimits,ylimits);
% warpedImg = imwarp(I1, trans,'OutputView',outputView);
% figure
% imshow(warpedImg)
% 
% figure
% I22 = imwarp(warpedImg,invert(trans));
% imshow(I22)
% figure
% imshow(I1)
red_mask = (I4(:,:,1) > 150) & (I4(:,:,2) < 80) & (I4(:,:,3) < 80);
blue_mask = (I4(:,:,1) < 60) & (I4(:,:,2) < 60) & (I4(:,:,3) > 100);
green_mask = (I4(:,:,1) < 40) & (I4(:,:,2) > 70) & (I4(:,:,3) < 60);
se_green = strel("disk",5);
se_blue = strel("disk",2);
se_red = strel("disk",5);
red_mask_dil = imerode(imclose(red_mask,se_red),se_red); %without this, the "redprop" will have lots of unnecessary coordinate, because the area is not connect owing to some small gaps in the image
blue_mask_dil = imerode(imclose(blue_mask,se_blue),se_blue);%so we have to put make those area as one area first, so the function "regionprops" will return one coordinate instead of several coordinates
green_mask_dil = imerode(imclose(green_mask,se_green),se_green);
%%find centers
redprop = regionprops(red_mask_dil,"Centroid");
redcenter = cat(1,redprop.Centroid);
world_red = transformPointsForward(tform,redcenter)
blueprop = regionprops(blue_mask_dil,"Centroid"); 
bluecenter = cat(1,blueprop.Centroid);
world_blue = transformPointsForward(tform,bluecenter)
greenprop = regionprops(green_mask_dil,"Centroid"); 
greencenter = cat(1,greenprop.Centroid);
world_green = transformPointsForward(tform,greencenter);% for path
world_corner = transformPointsForward(tform,sorted_all_center(1:2,:)')

figure
imshow(I4)
hold on
viscircles(bluecenter, 20, 'Color','g')
viscircles(redcenter, 20, 'Color','k')
viscircles(world_green, 20, 'Color','r')
hold on;
plot(bluecenter(:,1),bluecenter(:,2),"g*")
plot(redcenter(:,1),redcenter(:,2),"k*")
plot(greencenter(:,1),greencenter(:,2),"r*")
hold on

viscircles(bluecenter, 20, 'Color','g')
viscircles(redcenter, 20, 'Color','k')
viscircles(greencenter, 20, 'Color','r')
hold on;
plot(bluecenter(:,1),bluecenter(:,2),"g*")
plot(redcenter(:,1),redcenter(:,2),"k*")
plot(greencenter(:,1),greencenter(:,2),"r*")
hold on





I4 = warpedImg;
%% 原图像
red_mask = (I4(:,:,1) > 150) & (I4(:,:,2) < 80) & (I4(:,:,3) < 80);
blue_mask = (I4(:,:,1) < 60) & (I4(:,:,2) < 60) & (I4(:,:,3) > 100);
green_mask = (I4(:,:,1) < 40) & (I4(:,:,2) > 70) & (I4(:,:,3) < 60);
se_green = strel("disk",6);
se_blue = strel("disk",5);
se_red = strel("disk",6);
red_mask_dil = imerode(imclose(red_mask,se_red),se_red); %without this, the "redprop" will have lots of unnecessary coordinate, because the area is not connect owing to some small gaps in the image
blue_mask_dil = imerode(imclose(blue_mask,se_blue),se_blue);%so we have to put make those area as one area first, so the function "regionprops" will return one coordinate instead of several coordinates
green_mask_dil = imerode(imclose(green_mask,se_green),se_green);
%%find centers
redprop = regionprops(red_mask_dil,"Centroid");
redcenter = cat(1,redprop.Centroid);
%world_red = transformPointsForward(tform,redcenter)
blueprop = regionprops(blue_mask_dil,"Centroid"); 
bluecenter = cat(1,blueprop.Centroid);
%world_blue = transformPointsForward(tform,bluecenter)
greenprop = regionprops(green_mask_dil,"Centroid"); 
greencenter = cat(1,greenprop.Centroid);
%world_green = transformPointsForward(tform,greencenter);% for path
%world_corner = transformPointsForward(tform,sorted_all_center(1:2,:)')

[x,map] = imwarp(I3, trans);
figure
imshow(warpedImg)
hold on
viscircles(bluecenter, 20, 'Color','g')
viscircles(redcenter, 20, 'Color','k')
viscircles(world_green, 20, 'Color','r')
hold on;
plot(bluecenter(:,1),bluecenter(:,2),"g*")
plot(redcenter(:,1),redcenter(:,2),"k*")
plot(greencenter(:,1),greencenter(:,2),"r*")
hold on

viscircles(bluecenter, 20, 'Color','g')
viscircles(redcenter, 20, 'Color','k')
viscircles(greencenter, 20, 'Color','r')
hold on;
plot(bluecenter(:,1),bluecenter(:,2),"g*")
plot(redcenter(:,1),redcenter(:,2),"k*")
plot(greencenter(:,1),greencenter(:,2),"r*")
hold on

figure;
imshow(red_mask)
figure
imshow(blue_mask)
figure
imshow(green_mask)

% I3_fromI1 = imwarp(I3,tform);
% figure
% [x,map] = imwarp(I3,tform);
% imshow(x,map)
% imshow(I3_fromI1)
% 
% %% 一次tform变换后图像
% red_mask_I2_fromI1 = (I3_fromI1(:,:,1) > 150) & (I3_fromI1(:,:,2) < 80) & (I3_fromI1(:,:,3) < 80);
% blue_mask_I2_fromI1 = (I3_fromI1(:,:,1) < 80) & (I3_fromI1(:,:,2) < 80) & (I3_fromI1(:,:,3) > 140);
% green_mask_I2_fromI1 = (I3_fromI1(:,:,1) < 50) & (I3_fromI1(:,:,2) > 120) & (I3_fromI1(:,:,3) < 50);
% se_green_I2_fromI1 = strel("disk",6);
% se_blue_I2_fromI1 = strel("disk",5);
% se_red_I2_fromI1 = strel("disk",6);
% red_mask_dil_I2_fromI1 = imerode(imclose(red_mask_I2_fromI1,se_red_I2_fromI1),se_red_I2_fromI1); %without this, the "redprop" will have lots of unnecessary coordinate, because the area is not connect owing to some small gaps in the image
% blue_mask_dil_I2_fromI1 = imerode(imclose(blue_mask_I2_fromI1,se_blue_I2_fromI1),se_blue_I2_fromI1);%so we have to put make those area as one area first, so the function "regionprops" will return one coordinate instead of several coordinates
% green_mask_dil_I2_fromI1 = imerode(imclose(green_mask_I2_fromI1,se_green_I2_fromI1),se_green_I2_fromI1);
% %%find centers
% redprop_I2_fromI1 = regionprops(red_mask_dil_I2_fromI1,"Centroid");
% redcenter_I2_fromI1 = cat(1,redprop_I2_fromI1.Centroid);
% world_red_I2_fromI1 = transformPointsForward(tform,redcenter_I2_fromI1)
% blueprop_I2_fromI1 = regionprops(blue_mask_dil_I2_fromI1,"Centroid"); 
% bluecenter_I2_fromI1 = cat(1,blueprop_I2_fromI1.Centroid);
% world_blue_I2_fromI1 = transformPointsForward(tform,bluecenter_I2_fromI1)
% greenprop_I2_fromI1 = regionprops(green_mask_dil_I2_fromI1,"Centroid"); 
% greencenter_I2_fromI1 = cat(1,greenprop_I2_fromI1.Centroid);
% world_green_I2_fromI1 = transformPointsForward(tform,greencenter_I2_fromI1);% for path
% world_corner_I2_fromI1 = transformPointsForward(tform,sorted_all_center(1:2,:)')
% %%
% I3_onetform_colorcenter = [redcenter_I2_fromI1;bluecenter_I2_fromI1;greencenter_I2_fromI1];
% world_colorcenter = [world_red;world_blue;world_green]
% 
% world_and_trans_calculation = mean(world_colorcenter - I3_onetform_colorcenter)


%% Part B
bug2_map =  [ 
5 5 5 5 5 5 5 5 5 5
5 1 0 0 0 0 0 0 1 5
5 0 0 0 0 0 0 0 0 5
5 0 0 0 0 0 0 0 4 5
5 0 0 0 0 0 0 0 0 5
5 1 0 0 0 0 0 0 1 5
5 5 5 5 5 5 5 5 5 5 
];
% 读取图像
%I2 = warpedImg;
% 设置棋盘格子的行数和列数
numRows = 7;
numCols = 10;
% 计算每个方格的尺寸
[height, width, ~] = size(warpedImg);
gridHeight = floor(height / (numRows-2));
gridWidth = floor(width / (numCols-2));

[markerIds, markerCorners] = readArucoMarker(warpedImg, "DICT_4X4_250")

x_red = redcenter(:,1);
x_blue = bluecenter(:,1);
x_green = greencenter(:,1);
% 遍历每个方格
for row = 2:numRows-1
    for col = 2:numCols-1
        % 计算当前方格的边界
        rowStart = (row-1) * gridHeight + 1
        rowEnd = min(row * gridHeight, height)
        colStart = (col-1) * gridWidth + 1
        colEnd = min(col * gridWidth, width)
        for i = 1:length(markerIds)
            % 获取ArUco标记的中心点
            corners = markerCorners(:,:,i)
            center_corner(i,1) = mean(corners(:, 1)) % X 坐标的平均值
            center_corner(i,2) = mean(corners(:, 2)) % Y 坐标的平均值
            
            % 确定中心点所在的方格
            row_marker = ceil(center_corner(i,2) / gridHeight)
            col_marker = ceil(center_corner(i,1) / gridWidth)
            
            % 将对应方格的值设置为5
            bug2_map(row_marker, col_marker) = 5;
        end
        if bug2_map(row,col) == 3
            row_start = row;
            col_start = col;
        end
        for k = 1:size(x_red)
            row_red = ceil((redcenter(k,2)+gridHeight) / gridHeight)
            col_red = ceil((redcenter(k,1)+gridWidth) / gridWidth)
            bug2_map(row_red, col_red) = 1
        end
        for k = 1:size(x_blue)
            row_blue = ceil((bluecenter(k,2)+gridHeight) / gridHeight)
            col_blue = ceil((bluecenter(k,1)+gridWidth) / gridWidth)
            bug2_map(row_blue, col_blue) = 2
        end
        for k = 1:size(x_green)
            row_green = ceil((greencenter(k,2)+gridHeight) / gridHeight)
            col_green = ceil((greencenter(k,1)+gridWidth) / gridWidth)
            bug2_map(row_green, col_green) = 3;
        end
    end
end
center_corner5(1,1) = center_corner(1,1)+7*gridWidth
center_corner5(1,2) = center_corner(1,2)+4*gridHeight-20

%center_corner5 = [47 425];

center_corner6(1,1) = center_corner(1,1)+7*gridWidth
center_corner6(1,2) = center_corner(1,2)

%center_corner7 = [607 425];
center_corner7(1,1) = center_corner(1,1)
center_corner7(1,2) = center_corner(1,2)+4*gridHeight-20
center_corner8(1,1) = center_corner(1,1)
center_corner8(1,2) = center_corner(1,2)
all_center_corner = [center_corner5;center_corner6;center_corner7;center_corner8]
%all_center_corner = transformPointsForward(trans,allcorner)
figure
warpedImg = imwarp(I3, trans,'OutputView',outputView);
imshow(warpedImg)
axis on
hold on
plot(all_center_corner(:,1),all_center_corner(:,2),'r*')
before_scale_warpedimg_corner = all_center_corner;
world_corner_real = world_corner(5:8,:)
tform_new = fitgeotform2d(before_scale_warpedimg_corner, world_corner_real,'projective');
% find destination
[row,col] = find(bug2_map == 4);
unique_positions = unique([row, col], 'rows');
num_positions = size(unique_positions, 1);
for i = 1:num_positions
    goal_center_row(i) = unique_positions(i, 1) ; % 方块中心行位置
    goal_center_col(i) = unique_positions(i, 2) ; % 方块中心列位置
end
goal_center_row = mean(goal_center_row)
goal_center_col = mean(goal_center_col)

% find start position
[row,col] = find(bug2_map == 3);
unique_positions = unique([row, col], 'rows');
num_positions = size(unique_positions, 1);
for i = 1:num_positions
    start_center_row(i) = unique_positions(i, 1) ; % 方块中心行位置
    start_center_col(i) = unique_positions(i, 2) ; % 方块中心列位置
end
start_center_row = mean(start_center_row)
start_center_col = mean(start_center_col)
%
start = [start_center_col,start_center_row];
goal = [goal_center_col,goal_center_row];
bug2_map_copy = bug2_map;

bug2_map_copy(goal(2),goal(1)) = 0; %input is (rows,cols),and x is col, y is row, so
bug2_map_copy(start(2),start(1)) = 0;

% enlarge map
scale_factor = 10;
target_martrix = scale_factor*size(bug2_map)
resized_matrix = imresize(bug2_map, target_martrix, 'nearest'); % 使用 'nearest' 保持原始值

% find destination
[row,col] = find(resized_matrix == 4);
unique_positions = unique([row, col], 'rows');
num_positions = size(unique_positions, 1);
for i = 1:num_positions
    goal_center_row(i) = unique_positions(i, 1)+0.5 ; % 方块中心行位置
    goal_center_col(i) = unique_positions(i, 2)+0.5 ; % 方块中心列位置
end
goal_center_row = mean(goal_center_row)
goal_center_col = mean(goal_center_col)

% find start position
[row,col] = find(resized_matrix == 3);
unique_positions = unique([row, col], 'rows');
num_positions = size(unique_positions, 1);
for i = 1:num_positions
    start_center_row(i) = unique_positions(i, 1)+0.5 ; % 方块中心行位置
    start_center_col(i) = unique_positions(i, 2) +0.5; % 方块中心列位置
end
start_center_row = mean(start_center_row)
start_center_col = mean(start_center_col)
%
start = [start_center_col,start_center_row];
goal = [goal_center_col,goal_center_row];
bug = Bug2(resized_matrix);

% enlarge map
target_martrix = scale_factor*size(bug2_map)
resized_matrix_copy = imresize(bug2_map_copy, target_martrix, 'nearest')

se = strel('square', ceil(scale_factor));
boundary = resized_matrix_copy == 5;
obstacle_area = resized_matrix_copy == 1 | resized_matrix_copy == 2 | resized_matrix_copy ==5;
% 扩展障碍物区域
dilated_obstacles = imdilate(obstacle_area, se);

% 分别膨胀不同类型的障碍物
dilated_type1 = imdilate(resized_matrix_copy == 1, se);
dilated_type2 = imdilate(resized_matrix_copy == 2, se);
dilated_type3 = imdilate(resized_matrix_copy == 5, se);
% 将膨胀后的结果合并到一个矩阵中
dilated_matrix_withoutboundary = resized_matrix_copy; % 复制原始矩阵

% 用膨胀结果替换原矩阵中的值
dilated_matrix_withoutboundary(dilated_type1) = 1; % 用1表示类型1的障碍物
dilated_matrix_withoutboundary(dilated_type2) = 2; % 用2表示类型2的障碍物
dilated_matrix_withoutboundary(dilated_type3) = 5; % 用2表示类型2的障碍物
figure
bugpath = Bug2(dilated_matrix_withoutboundary);
%bugpath = Bug2(resized_matrix_copy)
bug.plot
colormap([1 1 1; 1 0 0; 0 0 1; 0 1 0; 1 1 1;1 0 0]); % 白色、红色、蓝色、绿色
grid on
hold on
bug_path=bugpath.query(start,goal);
plot(goal(1),goal(2),'k*')
hold on
plot(start(1),start(2),'k*')
hold on
plot(bug_path(:, 1), bug_path(:, 2),'LineWidth', 4, 'Color', 'g');

bug_path_middle(:, 1) = (bug_path(:, 1)/scale_factor)*gridWidth - gridWidth;
bug_path_middle(:, 2) = (bug_path(:, 2)/scale_factor)*gridHeight - gridHeight;
% bug_path_middle_3 = transformPointsInverse(trans,bug_path_middle)
% world_bug_path = transformPointsInverse(tform,bug_path_middle_3)
% point_start = world_bug_path(1,:);
world_bug_path = transformPointsForward(tform_new,bug_path_middle)
path = [world_bug_path,zeros(size(world_bug_path,1),1)];
traj = path;
for j = 1:length(path)
    %point = [[(traj(j,1:3) + [0, 0 30]),(home(4:6))],a,v,0,blend]
    point = [[(traj(j,1:3) + [0, 0 30]),(home(4:6))],a,v,0,blend]
    if j ==1
       paths = point
    else
       paths = cat(1,paths,point)
    end
end
poses = rtde.movej(home)
%vacuum.grip()
poses = rtde.movej(paths)
%vacuum.release()

% rtde.movej(home)
% startgrip = paths(1,:)-[0 0 25 0 0 0 0 0 0 0]
% rtde.movej(startgrip)
% pause(1)
% vacuum.grip()
% pause(1)
% rtde.movej(paths)
% vacuum.release()
% 
% rtde.drawPath(poses);
% [poses,joints,jointVelocities,jointAccelerations,torques] = rtde.movej(paths,'pose');
% rtde.movej(home)
% rtde.close;

% pause(1)
% rtde.movej(home)
% pause(1)
% startgrip = paths(1,:)-[0 0 23 0 0 0 0 0 0 0]
% rtde.movej(startgrip)
% disp("*")
% pause(1)
% vacuum.grip()
% pause(1)
% poses = rtde.movej(paths)
% endpoint = paths(end,:)-[0 0 23 0 0 0 0 0 0 0]
% rtde.movej(endpoint)
% vacuum.release()
% 
% rtde.drawPath(poses);
% [poses,joints,jointVelocities,jointAccelerations,torques] = rtde.movej(paths,'pose');
% rtde.movej(home)
% rtde.close;

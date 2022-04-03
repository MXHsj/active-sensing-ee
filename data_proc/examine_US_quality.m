%% US image quality experiment with active-sensing-ee
clc; clear; close all

file_dir = './data/clarius/';
experiment = 'Lung_Exam-01-Apr-2022/';
imgs = dir([file_dir, experiment, '*.jpeg']);

nimgs = length(imgs);
height = 768;       % clarius export setting
width = 1024;
bscan = zeros(height, width, nimgs, 'uint8');
for i=1:1:nimgs
   curr_file = imgs(i).name;
   curr_img = imread([file_dir,experiment,curr_file]);
   bscan(:,:,i) = rgb2gray(curr_img);
end

%% crop image
bscan_cropped = [];
% == crop frame
frm_top = 120; frm_bot = height - 120;
frm_left = 110; frm_right = width - 110;
% == remove clarius logo
logo_top = 20; logo_bot = 60; 
logo_left = 202; logo_right = 233;
for i = 1:nimgs
    curr_img = bscan(:,:,i);
    curr_img = curr_img(frm_top:frm_bot,frm_left:frm_right);
    curr_img(logo_top:logo_bot,logo_left:logo_right) = 0;
    bscan_cropped = cat(3,bscan_cropped,curr_img);
end

%% compare CNR
bscan_manual = bscan_cropped(:,:,1:2:end-1);
bscan_robot = bscan_cropped(:,:,2:2:end);
roi = cell(2,size(bscan_manual,3)); 
bg = cell(2,size(bscan_manual,3));
% manual ROI selection
doCNRComp = true;
updateRes = false;
if doCNRComp
    for i = 1:size(bscan_manual,3)
        fprintf('annotate manually collected image (%d/%d) \n',i,size(bscan_manual,3))
        imagesc(bscan_manual(:,:,i)); axis equal tight; colormap gray
        disp('draw roi'); 
        roi_v = drawrectangle('label','roi','LineWidth',1.5,'Color','red');
        input('press enter to continue')
        disp('draw background')
        bg_v = drawrectangle('label','bg','LineWidth',1.5,'Color','yellow');
        input('press enter to continue')
        roi_x=round(roi_v.Position(1)); bg_x=round(bg_v.Position(1));
        roi_y=round(roi_v.Position(2)); bg_y=round(bg_v.Position(2));
        roi_w=round(roi_v.Position(3)); bg_w=round(bg_v.Position(3));
        roi_h=round(roi_v.Position(4)); bg_h=round(bg_v.Position(4)); 
        roi{1,i} = bscan_manual(roi_y:roi_y+roi_h, roi_x:roi_x+roi_w,i);
        bg{1,i} = bscan_manual(bg_y:bg_y+bg_h, bg_x:bg_x+bg_w,i);
    end
    for i = 1:size(bscan_robot,3)
        fprintf('annotate robotically collected image (%d/%d) \n',i,size(bscan_manual,3))
        imagesc(bscan_robot(:,:,i)); axis equal tight; colormap gray
        disp('draw roi'); 
        roi_v = drawrectangle('label','roi','LineWidth',1.5,'Color','red');
        input('press enter to continue')
        disp('draw background')
        bg_v = drawrectangle('label','bg','LineWidth',1.5,'Color','yellow');
        input('press enter to continue')
        roi_x=round(roi_v.Position(1)); bg_x=round(bg_v.Position(1));
        roi_y=round(roi_v.Position(2)); bg_y=round(bg_v.Position(2));
        roi_w=round(roi_v.Position(3)); bg_w=round(bg_v.Position(3));
        roi_h=round(roi_v.Position(4)); bg_h=round(bg_v.Position(4)); 
        roi{2,i} = bscan_robot(roi_y:roi_y+roi_h, roi_x:roi_x+roi_w,i);
        bg{2,i} = bscan_robot(bg_y:bg_y+bg_h, bg_x:bg_x+bg_w,i);
    end
    if updateRes
        save([file_dir, experiment, 'roi.mat'],'roi');
        save([file_dir, experiment, 'bg.mat'],'bg')
    end
end

%% calculate CNR
CNR = zeros(2,size(bscan_manual,3));
for i = 1:size(bscan_manual,3)
    CNR(1, i) = abs(mean(roi{1,i},'all')-mean(bg{1,i},'all'))/ ...
        sqrt(var(double(roi{1,i}),1,'all')+var(double(bg{1,i}),1,'all'));
    CNR(2, i) = abs(mean(roi{2,i},'all')-mean(bg{2,i},'all'))/ ...
        sqrt(var(double(roi{2,i}),1,'all')+var(double(bg{2,i}),1,'all'));
end

figure('Position',[1920/3, 1080/3, 500, 400])
b = bar(CNR','BarWidth',0.8);
b(1).FaceColor = '#5b7888';
b(2).FaceColor = '#a6c9d7';
ax = get(gca); ax.YGrid = 'on';
xlabel('acquisition points'); ylabel('CNR')
hold on
legend('manual','robot','Location','northwest')

%% show example images
frm_id = 7;
figure('Position',[1920/4, 1080/3, 1200, 400])

subplot(1,2,1)
imagesc(bscan_manual(:,:,frm_id));
axis equal tight off
colormap gray
title(['freehand #',num2str(frm_id)])

subplot(1,2,2)
imagesc(bscan_robot(:,:,frm_id));
axis equal tight off
colormap gray
title(['robot #',num2str(frm_id)])


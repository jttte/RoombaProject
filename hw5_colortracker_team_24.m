%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 5 part1
%
% Team number: 24
% Team leader: Chia-Jung Lin (cl3295)
% Team members: Cheng Zhang (cz2398), Ming-Ching Chu (mc4107)
% 
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Command:
%    >> serPort = RoombaInit_mac ('usbserial');
%    >> hw5_colortracker_team_24(serPort);
% If you wish to downsample input image to further speed up processing, 
% please change the sample_rate up in line 22.
function hw5_colortracker_team_24(serPort)

    close all; clc;
    sample_rate = 1;
    img = imread('http://192.168.0.103/snapshot.cgi?user=admin&pwd=');
    
    
    h = ceil(size(img, 1)/sample_rate);
    l = ceil(size(img, 2)/sample_rate);
    
    angle_unit = 5;
    move_speed = 0.1;
    mid = ceil(l/2/sample_rate);
    
    % initialize
    fig = imshow(img);
    [y,x] = ginput(1)

    [c, area, center] = initialize_img_prop(img, x, y, l, h, sample_rate);
    
    last_area = area;
    while true
        img = imread('http://192.168.0.103/snapshot.cgi?user=admin&pwd=');
        [area, center] = process_img(img, c, l, h, sample_rate);
        if area == 0
            continue;
        end
        display(area);
        display(last_area);
        
        if abs(area-last_area) > 100          
            if area > last_area
                 travelDist (serPort, move_speed, -0.01);
                display('move back');
            else
                 travelDist (serPort, move_speed, 0.01);
                display('move forward');
            end
        end
        display(center);
        display(mid);
        if abs(center-mid) > 10
            if center > mid
                 turnAngle (serPort, 0.05, -angle_unit);
                display('turn right');
            else
                 turnAngle (serPort, 0.05, angle_unit);
                display('turn left');
            end
        end    
        pause(0.01);
    end
end


function [hue, area, center] = initialize_img_prop(img, x, y, l, h, sample_rate)
    new_img = zeros(h, l, 3);
    
%     tic
    new_img(:,:,1) = gaussfilt(mydownsample(img(:,:,1), sample_rate), 2);
    new_img(:,:,2) = gaussfilt(mydownsample(img(:,:,2), sample_rate), 2);
    new_img(:,:,3) = gaussfilt(mydownsample(img(:,:,3), sample_rate), 2);
%     toc
    
    hsv_img = rgb2hsv(new_img);
    
%     tic
    x = ceil(x/sample_rate);
    y = ceil(y/sample_rate);
    hue = hsv_img(x, y, 1);
    combined_mask = hsv_img(:,:,1) < hue + 0.06 & hsv_img(:,:,1) > hue - 0.06;
%     toc
    
%     tic
    % erode then label region
    L = bwlabel(imerode(combined_mask, [0 1 0; 1 1 1; 0 1 0]));
    % find label of the chosen region
    lable = L(x, y);
    target_mask = L == lable;
    figure,
    imshow(target_mask);
    area = sum(sum(target_mask));
    center = uint8(sum(target_mask * (1:l)') /area);
%     toc
end

function [area, center] = process_img(img, c, l, h, sample_rate)
    threshold = 0.06;
    new_img(:,:,1) = gaussfilt(mydownsample(img(:,:,1), sample_rate), 2);
    new_img(:,:,2) = gaussfilt(mydownsample(img(:,:,2), sample_rate), 2);
    new_img(:,:,3) = gaussfilt(mydownsample(img(:,:,3), sample_rate), 2);
    
    hsv_img = rgb2hsv(new_img);
    
    combined_mask = hsv_img(:,:,1) < c + threshold & hsv_img(:,:,1) > c - threshold;

    % erode then label region
    L = imerode(combined_mask, [0 1 0; 1 1 1; 0 1 0]);
    
    % connected components
    CC = bwconncomp(L);
    numPixels = cellfun(@numel,CC.PixelIdxList);
    
    % find the biggest CC
    [area,idx] = max(numPixels);
    if size(area, 2) == 0
        center = 0;
        return
    end
    result = zeros(h, l);
    result(CC.PixelIdxList{idx}) = 1;
    center = ceil(sum(result * (1:l)') /area);
    subplot(2,1,1), imshow(img)
    subplot(2,1,2), imshow(result)

end

function layer = mydownsample(img, sample_rate)
    img = (downsample(img', sample_rate))'; %down sample column
    layer = downsample(img, sample_rate); %down sample row
        
end

function smim = gaussfilt(im, sigma)
 
    assert(ndims(im) == 2, 'Image must be greyscale');
    
    % If needed convert im to double
    if ~strcmp(class(im),'double')
        im = double(im);  
    end
    
    sze = ceil(6*sigma);  
    if ~mod(sze,2)    % Ensure filter size is odd
        sze = sze+1;
    end
    sze = max(sze,1); % and make sure it is at least 1
    
    h = fspecial('gaussian', [sze sze], sigma);

    smim = filter2(h, im);
end
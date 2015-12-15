%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% COMS W4733 Computational Aspects of Robotics 2015
%
% Homework 5 part2
%
% Team number: 24
% Team leader: Chia-Jung Lin (cl3295)
% Team members: Cheng Zhang (cz2398), Ming-Ching Chu (mc4107)
%
% Input:
% serPort - Serial port object, used for communicating over bluetooth
%
% Command:
%    >> serPort = RoombaInit_mac ('usbserial');
%    >> hw5_doortracker_team_24(serPort);
% Since we only use low-level vision processing techniques, we recommand you 
% to set input image to the lowest resolution. The transmission can be 
% smoother this way. If you still have problem retrieving images, try
% connect camera to router using cable at all time.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function hw5_doortracker_team_24(serPort)
    close all; clc;
    img = read_img();

    h = size(img, 1);
    l = size(img, 2);
    angle_unit = 2;
    move_speed = 0.1;
    mid = ceil(l/2);

    % initialize
    thresh_1 = 4000;     % area below this value means door not found
    move_distance = 0.1; % no door in sight, just move forwards
    thresh_2 = 5;      % To align to the door, so that door is nearly in the mid
    thresh_3 = 5000;    % We are now in front of the door!
    thresh_orient = 1500;
    thresh_4 = 20;

    counter = 0;
    %Step1:  Turn around and move forward for the door!
    while true
        display('Part1');
            if counter < (360/(angle_unit*4)-6)
                img = read_img();
%                 [area, center] = process_img(img, c, l, h, sample_rate);
                [area, center] = process_img_gray(img, l, h);
                display(area)
                if area < thresh_1
                    turnAngle (serPort, 0.05, -angle_unit*4);
                    counter = counter + 1;
                else                      
                    break;
                end
            else
                % could not find object that is salient enough
                % turn to an orientation that is most likely toward doors
                % (having more dark areas) and then move forward
                while true
                    img = read_img();
                    total_area = get_object_area(img, l, h);
                    display(total_area)
                    if total_area > thresh_orient
                        break;
                    end
                    turnAngle (serPort, 0.05, -angle_unit);
                end
                travelDist (serPort, move_speed, move_distance*8);
                counter = 0;
            end                
    end
    
    % Step2:   Now we find a door! We need to align to it.
    while abs(center-mid) > thresh_2
        display('part2')
        if center > mid
             turnAngle (serPort, 0.05, -angle_unit);
            display('turn right');
        else
             turnAngle (serPort, 0.05, angle_unit);
            display('turn left');
        end
        img = read_img();
        [area, center] = process_img_gray(img, l, h);
    end 
    display(area);
    display(center);
    
    % Step3:  Move towards the door and justify the direction along the motion!
    last_area = area;
    last_center = center;
    while true
        display('part 4');
        travelDist (serPort, move_speed, move_distance);
        img = read_img();
        [area, center] = process_img_gray(img, l, h);
        display(area);
        if area > thresh_3
            break;
        end
        if area > last_area
            last_area = area;
            continue;       % We are now closer to the door! Move forword!
        else                % We are on the wrong direction, re-align
            while abs(center-mid) > thresh_2
                if abs(last_center - center) > thresh_4
                    display('reflection');
                    break;
                end
                if center > mid
                    turnAngle (serPort, 0.05, -angle_unit);
                    display('turn right');
                else
                    turnAngle (serPort, 0.05, angle_unit);
                    display('turn left');
                end
                img = read_img();
                [area, center] = process_img_gray(img, l, h);
                last_area = area;
                last_center = center;
                display(abs(last_center-center))
            end
        end
    end
    
    % We are now here to knock on the door and try to path it!
    i=0;
    while i<2
        display('part5')
        [BumpRight BumpLeft WheDropRight WheDropLeft WheDropCaster ...
        BumpFront] = BumpsWheelDropsSensorsRoomba(serPort);
        bumped= BumpRight || BumpLeft || BumpFront;
        if bumped
            travelDist (serPort, move_speed, - move_distance * 5);
            i = i+1;
        else
            travelDist (serPort, move_speed, move_distance);
        end
    end
    BeepRoomba(serPort);
    pause(5);
    travelDist (serPort, move_speed, move_distance * 10);
    
end

% count the total area of dark regions in the image
function total_area = get_object_area(img, l, h)
    gray_img = rgb2gray(img);
    
    cutoff_mask = zeros(h, l);
    cutoff_mask(1:floor(h/3), :) = 1;
    
    level = graythresh(gray_img);
    gray_mask = ~im2bw(gray_img, level);
    combined_mask = cutoff_mask & gray_mask;
    
    % erode then label region
    L = imerode(combined_mask, [0 1 0; 1 1 1; 0 1 0]);
    
    total_area = sum(sum(L));
end

function [area, center] = process_img_gray(img, l, h)
    gray_img = rgb2gray(img);
    
    % we only look at the top 1/3 part of the photo
    % so that pattern on the floor will not interfere our processing
    cutoff_mask = zeros(h, l);
    cutoff_mask(1:floor(h/3), :) = 1;
    
    % histogram normalization, change threshold dynamically
    % we are interested in the darker region
    level = graythresh(gray_img);
    gray_mask = ~im2bw(gray_img, level);
    combined_mask = cutoff_mask & gray_mask;
    
    % erode then label region
    L = imerode(combined_mask, [0 1 0; 1 1 1; 0 1 0]);
    
    % connected components
    CC = bwconncomp(L);
    numPixels = cellfun(@numel,CC.PixelIdxList);
    
    % find the biggest CC
    [area,idx] = max(numPixels);
    if size(area, 2) == 0
        center = 0;
        area = 0;
        return
    end
    result = zeros(h, l);
    result(CC.PixelIdxList{idx}) = 1;
    size(result)
    area
    temp = sum(result * (1:l)')/area;
    center = ceil(temp);
    subplot(2,1,1), imshow(img)
    subplot(2,1,2), imshow(result)
    
end

function img = read_img()
    img = imread('http://192.168.0.102/snapshot.cgi?user=admin&pwd=');
end

% function layer = mydownsample(img, sample_rate)
%     img = (downsample(img', sample_rate))'; %down sample column
%     layer = downsample(img, sample_rate); %down sample row
%         
% end
% 
% function smim = gaussfilt(im, sigma)
%  
%     assert(ndims(im) == 2, 'Image must be greyscale');
%     
%     % If needed convert im to double
%     if ~strcmp(class(im),'double')
%         im = double(im);  
%     end
%     
%     sze = ceil(6*sigma);  
%     if ~mod(sze,2)    % Ensure filter size is odd
%         sze = sze+1;
%     end
%     sze = max(sze,1); % and make sure it is at least 1
%     
%     h = fspecial('gaussian', [sze sze], sigma);
% 
%     smim = filter2(h, im);
% end
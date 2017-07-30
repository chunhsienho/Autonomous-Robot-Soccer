
%%

clc
%%
%initiate camera and Bluetooth
vid = videoinput('winvideo', 1, 'I420_800x600');%I420_800x600'
vid.ReturnedColorspace = 'rgb';

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Only need to trigger once
vid.FramesPerTrigger = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
resize_ratio = 0.4;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
triggerconfig(vid, 'manual');

BT = visa('ni', 'ASRL7::INSTR');    %%COM() = ASRL()

fopen(BT);
BT.status;
% Tx= 'r';
% fprintf(BT,Tx);
% pause(0.4);    %%changable, used to be 0.5
% Tx= 'I';
% fprintf(BT,Tx);
%%
% Parameters



% Morphology element
%%%%%%%%%%%%%%%%%%%%%%%%%%
% se_1 = strel('disk',5);
se_1 = strel('disk',2);

% Unit vectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ux = [1;0];
uy = [0;1];
timetimetime=0;
% Parameters of car
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
alpha = 0.2;

% Plotting parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Marker_size = 15;



global bool; 
global output; %output information 

% Strategy parameters
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R_end = 13;%%  §PÂ_¬O§_¨ì¦ì
A_end = 6;%%  §PÂ_¬O§_¨ì¦ì
R_BALL_ACC=50;  %%
V_CATCH=380;   %%±close ball
V_MAX=350; %%max value for vehicle
X_front_gate = 270;   %%
Y_front_gate = 120;
X_initial = 80;
Y_initial = 120;




%%%%%%%%%% avoid use %%%%%%%%%%
perpendVec=  zeros(2,1);
avoDist= 55;%distance to avoid other car (side move)
avoDist2=35;%distance to avoid other car(back move)           %####################################################§ï
rightLeft = 1; %right for positive,left for negative
warnBre = 45;  %thershold for choosing the road,if the number is lower than is number than change other road
givUpL = 30;   %
distK = 50;    % The desire angle for change to avoidance mode
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%Stuck Mode
%%%%%%%%%% avoid stucking use %%%%%%%%%%

car_1_old = zeros(2,1);
stuckWidth = 50;
timeNow = 0;
timeGo =0;
STUCK_TIME = 18;
GO_TIME = 10;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%Case for Car
bool.ballGet=0; %%get the ball or not
bool.reverseGoal=0; %%
bool.blockGoal=0;  %% check if there is any opponent on the desire road
bool.rushGoal=0; %%Defense 
bool.backAll=0; %% 
bool.pause=0;
bool.avoid=0; %%Avoidance function activate or not


% Output via Bluetooth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
output.EndX=0;
output.EndY=0;
output.EndD=200;  %%for initially give a speed
output.EndA=0;

output.V2=0;
output.claimBallNew=0;
output.claimBallOld=0;







%%

% Start the cammera
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
start(vid)

pic = imresize(getsnapshot(vid),1,'nearest');
h = imshow(pic);

tic
time = 0;
while time < 0.3;
    time = toc;
end
% Initialize the object of display

hold on
h1 = plot(1,1,'o','MarkerSize',Marker_size,'MarkerEdgeColor','r');
h2 = plot(1,1,'o','MarkerSize',Marker_size,'MarkerEdgeColor','g');
h3 = plot(1,1,'o','MarkerSize',Marker_size,'MarkerEdgeColor','b');
h4 = plot(1,1,':y');
h5 = plot(1,1,'square','MarkerSize',5,'MarkerEdgeColor','c');
h6 = plot(1,1,':y');%line
h7 = plot(1,1,'.','MarkerSize',Marker_size,'MarkerEdgeColor','k');
h8 = plot(1,1,'.','MarkerSize',Marker_size,'MarkerEdgeColor','w');
h9 = plot(1,1,':k');%line
h10 = plot(1,1,':w');%line
h11 = plot(1,1,'o','MarkerSize',Marker_size,'MarkerEdgeColor','y');
h12 = plot(1,1,'o','MarkerSize',Marker_size,'MarkerEdgeColor','m');
h13 = plot(1,1,':y');%line
h_pause = plot(1,1,'square','MarkerSize',Marker_size,'MarkerEdgeColor','w');

h_text1 =text(1,1,'');%car
h_text2 =text(1,1,'');%car and ball
h_text3 =text(1,1,'');%ball and black goal
h_text4 =text(1,1,'');%ball and white goal
h_text5 =text(1,1,'');%car2
h_text6 =text(1,1,'');%car2 and ball
h_text7 =text(1,1,'');%car2 and car1
h_title = title('Frame number:0');
hold off

set(h1,'visible','off')
set(h2,'visible','off')
set(h3,'visible','off')
set(h4,'visible','off')
set(h5,'visible','off')
set(h6,'visible','off')
set(h7,'visible','off')
set(h8,'visible','off')
set(h9,'visible','off')
set(h10,'visible','off')
set(h11,'visible','off')
set(h12,'visible','off')
set(h13,'visible','off')

set(h_text1,'visible','off')
set(h_text2,'visible','off')
set(h_text3,'visible','off')
set(h_text4,'visible','off')
set(h_text5,'visible','off')
set(h_text6,'visible','off')
set(h_text7,'visible','off')
set(h_pause,'visible','off')


% initialization of valuables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
frame_num = 0;

%color
Appear_red = 0;
Appear_green = 0;
Appear_blue = 0;
Appear_purple = 0;
Appear_yellow = 0;

%car1
p_car1_rear = zeros(2,1);
p_car1_front = zeros(2,1);
centroid_car_1 = zeros(2,1);
dir_car_1 = ux;
length_car_1 = 1;
ang_car_1 = 0;
%ball
p_ball = zeros(2,1);
dir_ball_1 = ux;
dir_ball_2 = ux;
distance_ball_1 = 1;%for ball and car1
distance_ball_2 = 1;%for ball and car2

ang_rel_ball_1 = 0;%car1
ang_rel_ball_2 = 0;%for car with ball and black
ang_rel_ball_3 = 0;%for car with ball and white
ang_rel_ball_4 = 0;%car2
ang_rel_ball_5 = 0;%for car2 and black
%car2 (enemy)
p_car2_rear = zeros(2,1);
p_car2_front = zeros(2,1);
centroid_car_2 = zeros(2,1);
dir_car_2 = ux;
length_car_2 = 1;
ang_car_2 = 0;

dircardistance = zeros(2,1);
cardistance = 1;
ang_car = 0;

%door
dir_door_1 = ux; %black
dir_door_2 = ux; %white
door_location_1 = [20,120]; %black  %%defense
door_location_2 = [300,120]; %white  %%attack
distance_door_2 =1;%black
distance_door_3 =1;%white
distance_door_4 =1;%car2 and black

door_length1=1;%black
door_length2=1;%white
door_length3=1;%car2 and black

B=[];%car rear
C=[];%car front
D=[];%ball
E=[];%angle
F=[];%distance

% Nominal hue of each color
%%%%%%%%%%%%%%%%%%%%%%%%%%
hue_red = 0.02; %0.02;

hue_blue = 0.62;  %0.66;
hue_green = 0.485; % 0.33;
hue_yellow = 0.12;%0.17
hue_purple = 0.9;%0.95
hue_pause = 0.8399;

% Tolerance of hue
%%%%%%%%%%%%%%%%%%%%%%%%%%
hue_tolerance = 0.0417; % 0.0417 = (1/6)/2/2;

sat_red_min = 0.6;
sat_green_min = 0.49;
sat_blue_min = 0.4;
sat_yellow_min = 0.4;
sat_purple_max = 0.6;%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sat_purple_min = 0.3;
sat_pause_min = 0.3;
% Maximun satuation of each color
%%%%%%%%%%%%%%%%%%%%%%%%%%
sat_green_max = 1;

B=[];%car rear
C=[];%car front
D=[];%ball
E=[];%angle
F=[];%distance
N=[];

while 1;     % ???????Command window ?? ctrl + c ????? stop(vid);!!
    
    tic
    pic = imresize(peekdata(vid,1), 0.4, 'nearest');
    frame_num = frame_num + 1;   
    % Image recognition process
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Project 6 algorithm

    %%
    % Image recognition process
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Transfer the pic from RGB to HSV color space
    pic_hsv = rgb2hsv(pic);
    pic_h = pic_hsv(:,:,1);
    pic_s = pic_hsv(:,:,2);
    pic_v = pic_hsv(:,:,3);
    
    % Nominal hue of each color
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    % Thresholding by hue
    % Thresholding by hue
    BW_h_mask_red = pic_h <= (hue_red + hue_tolerance) | pic_h >= (1+ hue_red - hue_tolerance);%
    BW_h_mask_green = (pic_h <= (hue_green + hue_tolerance)) & (pic_h >= (hue_green - hue_tolerance));%
    BW_h_mask_blue = (pic_h <= (hue_blue + hue_tolerance)) & (pic_h >= (hue_blue - hue_tolerance));%
    BW_h_mask_pause = (pic_h <= (hue_pause + hue_tolerance)) & (pic_h >= (hue_pause - hue_tolerance));%
    BW_h_mask_yellow = (pic_h <= (hue_yellow + hue_tolerance)) & (pic_h >= (hue_yellow - hue_tolerance));%
    BW_h_mask_purple = (pic_h <= (hue_purple + hue_tolerance)& pic_h > 0.7 );%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Minimun satuation of each color
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Thresholding by satuation
    BW_s_mask_red = pic_s >= sat_red_min;
    BW_s_mask_green = pic_s >=sat_green_min & pic_s <= sat_green_max;
    BW_s_mask_blue = pic_s >= sat_blue_min;
    BW_s_mask_pause = pic_s >= sat_pause_min;
    BW_s_mask_yellow = pic_s >= sat_yellow_min;
    BW_s_mask_purple = (pic_s <= sat_purple_max)&(pic_s >= sat_purple_min);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Thresholding by value
    BW_v_mask = pic_v >=0.2;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Get the specific-color binary image
    BW_red = BW_h_mask_red & BW_s_mask_red & BW_v_mask;
    BW_green = BW_h_mask_green & BW_s_mask_green & BW_v_mask;
    BW_blue = BW_h_mask_blue & BW_s_mask_blue & BW_v_mask;
    BW_pause = BW_h_mask_pause & BW_s_mask_pause & BW_v_mask;
    BW_yellow = BW_h_mask_yellow & BW_s_mask_yellow & BW_v_mask;
    BW_purple = BW_h_mask_purple & BW_s_mask_purple & BW_v_mask;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Thresholding by value
    BW_v_mask_green = pic_v >=0.15;

    %%
    % Morphology element
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    se_1 = strel('disk',3);
    
    % Morphology open and close
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ?????
    BW_red_close = imclose(BW_red,se_1);
    BW_red_open = imopen(BW_red_close,se_1);
    BW_green_close = imclose(BW_green,se_1);
    BW_green_open = imopen(BW_green_close,se_1);
    BW_blue_close = imclose(BW_blue,se_1);
    BW_blue_open = imopen(BW_blue_close,se_1);
    BW_pause_close = imclose(BW_pause,se_1);
    BW_pause_open = imopen(BW_pause_close,se_1);
    BW_yellow_close = imclose(BW_yellow,se_1);
    BW_purple_close = imclose(BW_purple,se_1);
    BW_yellow_open = imopen(BW_yellow_close,se_1);
    BW_purple_open = imopen(BW_purple_close,se_1);
    
    % Centroids calculation ????
    S_r = regionprops(BW_red_open,'Centroid');
    S_g = regionprops(BW_green_open,'Centroid');
    S_b = regionprops(BW_blue_open,'Centroid');
    S_y = regionprops(BW_yellow_open,'Centroid');
    S_p = regionprops(BW_purple_open,'Centroid');
    S_pause = regionprops(BW_pause_open,'Centroid');
    
    centroids_red = cat(1, S_r.Centroid);
    centroids_green = cat(1, S_g.Centroid);
    centroids_blue = cat(1, S_b.Centroid);
    centroids_yellow = cat(1, S_y.Centroid);
    centroids_purple = cat(1, S_p.Centroid);
    centroids_pause = cat(1, S_pause.Centroid);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % End of image recognition process
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % End of image recognition process
    
    % Appearance of markers
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Navigational indexes
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Cauculate the location and the orientation of cars
    Appear_red = (size(centroids_red,1) ~= 0);
    Appear_green = (size(centroids_green,1) ~= 0);
    Appear_blue = (size(centroids_blue,1) ~= 0);
    Appear_purple = (size(centroids_purple,1) ~= 0);
    Appear_yellow = (size(centroids_yellow,1) ~= 0);
    Appear_pause = (size(centroids_pause,1) ~= 0);

    % Navigational indexes
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Cauculate the location and the orientation of cars
    if Appear_yellow && Appear_purple;
        %car2
        p_car2_rear = centroids_red(1,:).';
        p_car2_front = centroids_green(1,:).'; %
        dir_car_2 = p_car2_front - p_car2_rear;
        length_car_2 = norm(dir_car_2,2);
        
        if dir_car_2(2,1)>=0;
            ang_car_2 = 180/pi*acos(dir_car_2.'*ux/length_car_2);
        else
            ang_car_2 = -180/pi*acos(dir_car_2.'*ux/length_car_2);
        end
        
        centroid_car_2 = p_car2_rear + alpha*dir_car_2;
    end
    if Appear_red && Appear_green; %
        %car1
        p_car1_rear = centroids_yellow(1,:).'; %
        p_car1_front = centroids_purple(1,:).'; %
        dir_car_1 = p_car1_front - p_car1_rear;
        length_car_1 = norm(dir_car_1,2);
        
        
        
        if dir_car_1(2,1) >= 0;
            ang_car_1 = 180/pi*acos(dir_car_1.'*ux/length_car_1);
            
        else
            ang_car_1 = -180/pi*acos(dir_car_1.'*ux/length_car_1);
            
        end

        % Centroid of the car
        centroid_car_1 = p_car1_rear + alpha*dir_car_1;
        
        dircardistance = centroid_car_2- centroid_car_1;
        cardistance = norm(dircardistance);
        
        if det([dir_car_1.';dircardistance.']) >= 0; % "cross"
            ang_car = -180/pi*acos(dir_car_1.'*dircardistance/length_car_1/cardistance);
            
        else
            ang_car = 180/pi*acos(dir_car_1.'*dircardistance/length_car_1/cardistance);
            
        end
        
        
        if det([dir_car_1.';dir_door_1.']) >= 0;
            ang_rel_ball_2 = -acos(dir_car_1.'*dir_door_1/length_car_1/distance_door_2)*180/pi;
        else
            ang_rel_ball_2 = 180/pi*acos(dir_car_1.'*dir_door_1/length_car_1/distance_door_2);
        end
        
        if det([dir_car_1.';dir_door_2.']) >= 0;
            ang_rel_ball_3 = -acos(dir_car_1.'*dir_door_2/length_car_1/distance_door_3)*180/pi;
        else
            ang_rel_ball_3 = 180/pi*acos(dir_car_1.'*dir_door_2/length_car_1/distance_door_3);
            
        end
        
        %%%%%%%%%%%%%%%
        % Cauculate the location of the ball
        % Relative position of the ball
        
        if Appear_blue; %
            p_ball = centroids_blue(1,:).'; %
            dir_ball_1 = p_ball - centroid_car_1;
            dir_ball_2 = p_ball - centroid_car_2;
            
            distance_ball_1 = norm(dir_ball_1,2);
            distance_ball_2 = norm(dir_ball_2,2);
            
            dir_door_1 = door_location_1'-centroid_car_1;
            dir_door_2 = door_location_2'-centroid_car_1;
            dir_door_3 = door_location_1'-centroid_car_2;
            
            distance_door_2=  norm(dir_door_1,2);
            distance_door_3 = norm(dir_door_2,2);
            distance_door_4 = norm(dir_door_3,2);
            
            
            
            door_length1 = norm(door_location_1,2);
            door_length2 = norm(door_location_2,2);
            door_length3 = norm(door_location_1,2);
            
            if det([dir_car_1.';dir_ball_1.']) >= 0; % "cross"
                ang_rel_ball_1 = -180/pi*acos(dir_car_1.'*dir_ball_1/length_car_1/distance_ball_1);
                
            else
                ang_rel_ball_1 = 180/pi*acos(dir_car_1.'*dir_ball_1/length_car_1/distance_ball_1);
                
            end

            %%not suitable to put here, should put in the "appear
            %%dir_car_2",                                             STEVE
            if det([dir_car_2.';dir_ball_1.']) >= 0; % "cross"
                ang_rel_ball_4 = -180/pi*acos(dir_car_2.'*dir_ball_1/length_car_2/distance_ball_1);
                
            else
                ang_rel_ball_4 = 180/pi*acos(dir_car_2.'*dir_ball_1/length_car_2/distance_ball_1);
                
            end
            
            
            if det([dir_car_2.';dir_door_1.']) >= 0;
                ang_rel_ball_5 = -acos(dir_car_2.'*dir_door_1/length_car_2/distance_door_4)*180/pi;
            else
                ang_rel_ball_5 = 180/pi*acos(dir_car_2.'*dir_door_1/length_car_2/distance_door_4);
            end
            
        end

    end
    A= p_car1_front;
    G= p_car1_rear;
    H= ang_rel_ball_1;
    I= distance_ball_1;
    J= p_ball;
    B=[B,A];
    C=[C,G];
    D=[D,J];
    E=[E,H];
    F=[F,I];

    %% AI Structure
    %%check the speed
    if distance_ball_1 < R_BALL_ACC && bool.ballGet == 0 %??????
        output.V2= V_CATCH;		%%slow down
    elseif output.EndD < R_end && abs(output.EndA) < A_end
        output.V2= 0;		
        %     elseif output.EndD < 25 && bool.ballGet == 1
        %         output.V2=0;
    else
        output.V2= V_MAX;
    end
    
    
    %check if get the ball or not
    if (abs(ang_rel_ball_1)<18) && distance_ball_1<= 35
        bool.ballGet = 1; %%true
        
    else
        bool.ballGet = 0;%%false
    end
    
    
    if bool.avoid==0   %%did not need to avoidance
  %4
        if bool.ballGet == 0  %did not get ball
            
            if bool.backAll==1  %defense
                output.EndD=(distance_door_2);
                output.EndA=ang_rel_ball_2;
                output.claimBallNew=0;
            else    %chase ball
                
                output.EndD=(distance_ball_1);
                output.EndA=ang_rel_ball_1;
                output.claimBallNew=0;
            end
            
        else  %with ball and would like to goal
            
            if centroid_car_1(1)>250 && centroid_car_1(2) >120-25 && centroid_car_1(2) < 120+25
                %¦A©¹ªù¤f½Ä            (2)
                output.EndD=(distance_door_3);
                output.EndA=ang_rel_ball_3;
                
                if abs(ang_rel_ball_3)<10 && centroid_car_1(1)>270 && abs(ang_car_1) <15    %%¨¬°÷±µªñ¥H¤Î¨¤«×¹ï¨ì¤F¡A¥i¥H¥´¶}§¨§ì
                    output.claimBallNew=0;
                else
                    output.claimBallNew=1;
                end
                
            else
                %¥ý¶]¨ìªù¤f«e­±ªº¦ì¸m   (1)
                output.EndX= X_front_gate;
                output.EndY=Y_front_gate;
                
                if centroid_car_2(1)>250 && centroid_car_2(2) >120-25 && centroid_car_2(2) < 120+25
                    if centroid_car_2(2)>120                    
                        output.EndX=300;
                        output.EndY=135;
                    else                                            
                        output.EndX=300;
                        output.EndY=105;
                    end
                end
                
                tempRelEnd = [output.EndX - centroid_car_1(1) ;output.EndY - centroid_car_1(2)];
                output.EndD = norm(tempRelEnd,2);
                if det([dir_car_1.';tempRelEnd.']) >= 0; % "cross"
                    output.EndA = -180/pi*acos(dir_car_1.'*tempRelEnd/length_car_1/output.EndD);
                else
                    output.EndA = 180/pi*acos(dir_car_1.'*tempRelEnd/length_car_1/output.EndD);
                end
                output.claimBallNew=1;
               
                
            end
            
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  Structure
        %1.Have Stop sign or not? If have stop sign then stop
        %2.Be stuck at the corner or wall?
        %3.See the ball or not
        %4.Get the ball or not
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     %1 %if find the purple,then set the vehicle velocity to be zero
    if Appear_pause
        output.V2=0;
    end
    
    %2    %%Stuck by the wall or corner start: Be stuck at the corner or wall?
    
    if(bool.avoid ~= 2 && Appear_pause ==0 && Appear_blue) 
        if (norm(car_1_old - centroid_car_1,2) < 6)
            timeNow = timeNow +1;
            
        else %%«ì´_¥¿±`¡A§YÂk¹s
            timeNow = 0;
        end
        
        if timeNow == STUCK_TIME  %«á°h
            timeNow =0;
            bool.avoid=2;
        end
    end
    
 %3 See the ball or not
    if Appear_blue ==0   %%no ball
            
        output.EndX=X_initial;
        output.EndY=Y_initial;
            tempRelEnd = [output.EndX - centroid_car_1(1) ;output.EndY - centroid_car_1(2)];
            output.EndD = norm(tempRelEnd,2);
        if det([dir_car_1.';tempRelEnd.']) >= 0; % "cross"
                output.EndA = -180/pi*acos(dir_car_1.'*tempRelEnd/length_car_1/output.EndD);
        else
                output.EndA = 180/pi*acos(dir_car_1.'*tempRelEnd/length_car_1/output.EndD);
        end
            output.claimBallNew=0;
            
            %         if output.EndD > R_end
            %             if abs(output.EndA) > 110
            %                 output.V2 = -1*output.V2;
            %             end
            %         else
        if output.EndD < 25
                output.EndD = 0;
                output.EndA=ang_rel_ball_3;
                output.V2= 150;
            if abs(output.EndA) < 12
                    output.V2= 0;
            end                
        end
    end
        %

        %Avoidance~~~ Á×»Ù
        %testing use distance 30, warning if dist_car <30, will occur error
        %abcccc= asind(30/cardistance);   %%fortesting
        %%10«× *distK/cardistance ¬O¬°¤F°µ¦¨¦pªG¤Óªñ¡A«h¨¤«×­­¨îÅÜ¤p
        if(Appear_yellow && Appear_purple && abs(ang_car-output.EndA) < 20 && bool.avoid == 0 && output.EndD > cardistance )  %%##############team B ­n§ï
            perpendVec = [dircardistance(2) ; -1*dircardistance(1)] / cardistance * avoDist;
            negVec= dircardistance/ cardistance *avoDist2;
            
            if (centroid_car_2(1) - negVec(1) + perpendVec(1)> warnBre && centroid_car_2(1)- negVec(1) + perpendVec(1) < 310 - warnBre) && (centroid_car_2(2) - negVec(2) + perpendVec(2) >warnBre && centroid_car_2(2) - negVec(2) + perpendVec(2) < 240 - warnBre)|| ((centroid_car_2(1) - negVec(1)- perpendVec(1) > warnBre && centroid_car_2(1)- negVec(1) - perpendVec(1) < 310 - warnBre) && (centroid_car_2(2) - negVec(2) - perpendVec(2) >warnBre && centroid_car_2(2) - negVec(2) - perpendVec(2) < 240 -warnBre ))
                bool.avoid=1;
            end
            %             if (centroid_car_2(1) - negVec(1) + perpendVec(1)> warnBre &&
            %             centroid_car_2(1)- negVec(1) + perpendVec(1) < 310 - warnBre)
            %             && (centroid_car_2(2) - negVec(2) + perpendVec(2) >warnBre &&
            %             centroid_car_2(2) - negVec(2) + perpendVec(2) < 240 - warnBre
            %             )
            %                 rightLeft = 1;
            %
            %             elseif (centroid_car_2(1) - negVec(1)- perpendVec(1) > warnBre && centroid_car_2(1)- negVec(1) - perpendVec(1) < 310 - warnBre) && (centroid_car_2(2) - negVec(2) - perpendVec(2) >warnBre && centroid_car_2(2) - negVec(2) - perpendVec(2) < 240 -warnBre )
            %                 rightLeft = -1;
            %             else
            %                 bool.avoid=0;
            %             end
            
            if output.EndA > ang_car
                rightLeft = 1;
            else
                rightLeft = -1;
            end
        end
        
        %%%
    elseif bool.avoid==1 %%»Ý­nÁ×»Ù
        
        %if elseªº§P©w¬O¥ý½T©w¸ô®|ªº¥Ø¼ÐÂI¬O§_¦b¦w¥þ½d³ò¥H¤º
        if centroid_car_2(1) - negVec(1) + rightLeft*perpendVec(1)> warnBre && centroid_car_2(1)- negVec(1) + rightLeft*perpendVec(1) < 310 - warnBre && centroid_car_2(2) - negVec(2) + rightLeft*perpendVec(2) >warnBre && centroid_car_2(2) - negVec(2) + rightLeft*perpendVec(2) < 240 - warnBre
            
        elseif centroid_car_2(1) - negVec(1)- rightLeft*perpendVec(1) > warnBre && centroid_car_2(1)- negVec(1) - rightLeft*perpendVec(1) < 310 - warnBre && centroid_car_2(2) - negVec(2) - rightLeft*perpendVec(2) >warnBre && centroid_car_2(2) - negVec(2) - rightLeft*perpendVec(2) < 240 -warnBre && cardistance > givUpL
            rightLeft = rightLeft*-1;
        else
            bool.avoid=0;
        end
        
        output.EndX= centroid_car_2(1) - negVec(1)+ rightLeft*perpendVec(1);
        output.EndY= centroid_car_2(2) + rightLeft*perpendVec(2)- negVec(2);
        tempRelEnd = [output.EndX - centroid_car_1(1) ;output.EndY - centroid_car_1(2)];
        output.EndD = norm(tempRelEnd,2);
        if det([dir_car_1.';tempRelEnd.']) >= 0; % "cross"
            output.EndA = -180/pi*acos(dir_car_1.'*tempRelEnd/length_car_1/output.EndD);
        else
            output.EndA = 180/pi*acos(dir_car_1.'*tempRelEnd/length_car_1/output.EndD);
        end
        
        if bool.ballGet==0
            if abs(ang_car-ang_rel_ball_1) >= 45 || distance_ball_1 < cardistance + 5
                bool.avoid=0;
            end
            
        else
            if abs(ang_car-ang_rel_ball_3) >= 45 || distance_door_3 < cardistance + 5                
                bool.avoid=0;
            end
        end
        
        if output.EndD<23   %%¨ì¹F¥Ø¼ÐÂI
            
            bool.avoid=0;
        end
        
    elseif bool.avoid==2
       
        
        timeGo= timeGo + 1;
        
        output.EndD = 1000;
        output.EndA = 179;
        output.V2= -300;
        
        if bool.ballGet ==0
            if timeGo < GO_TIME/2
                output.claimBallNew=1;
            else
                output.claimBallNew=0;
            end            
        end
        
        if timeGo == GO_TIME
            
            timeGo=0;
            bool.avoid=0;
            
        end

        
    end
  
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %     output.EndD = -1000;
    %     output.EndA = 0;

    %     output.V2 = -104.5;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    car_1_old = centroid_car_1;
    Tx = '$';
    EEndD = int16(output.EndD);
    EEndA = int16(output.EndA);
    VV2 = int16(output.V2);
    
    %%Distance to string     %%¦h¤F¥¿­t¸¹
    if output.EndD >=0;
        Tx = [Tx '0'];
    else %­tªº
        Tx = [Tx '-'];
        EEndD = -1*EEndD;
    end
    
    if EEndD>=1000;
        Tx = [Tx sprintf('%d',EEndD)];
    elseif EEndD>=100;
        Tx = [Tx sprintf('0%d',EEndD)];
    elseif EEndD>=10;
        Tx = [Tx sprintf('00%d',EEndD)];
    else
        Tx = [Tx sprintf('000%d',EEndD)];
    end
    
    %%Angle to string
    if output.EndA >=0;
        Tx = [Tx '0'];
        angle_temp = EEndA;
    else %­tªº
        Tx = [Tx '-'];
        angle_temp = -1*EEndA;
    end
    
    if angle_temp >=100;
        Tx = [Tx sprintf('%d',angle_temp)];
    elseif angle_temp >=10;
        Tx = [Tx sprintf('0%d',angle_temp)];
    else
        Tx = [Tx sprintf('00%d',angle_temp)];
    end
    
    %Velocity
    if output.V2 >=0;
        Tx = [Tx '0'];
        velocity_temp = VV2;
    else %­tªº
        Tx = [Tx '-'];
        velocity_temp = -VV2;
    end
    
    if velocity_temp >=100;
        Tx = [Tx sprintf('%d',velocity_temp)];
    elseif velocity_temp >=10;
        Tx = [Tx sprintf('0%d',velocity_temp)];
    else
        Tx = [Tx sprintf('00%d',velocity_temp)];
    end
    
    
    %%instruction "m"¦
    Tx = [Tx 'm'];
    fprintf(BT,Tx);
    
    
    if bool.pause==1
        pause(1.5);
        bool.pause =0;
    end
    
    %%scrach function
    if output.claimBallNew ~= output.claimBallOld
        if output.claimBallNew==1
            Tx= 'c';
        elseif output.claimBallNew==0
            Tx= 'r';
        end
        fprintf(BT,Tx);
        %         pause(0.4);    %%changable, used to be 0.5
        %         Tx= 'I';
        %         fprintf(BT,Tx);
    end
    
    output.claimBallOld = output.claimBallNew;
    %%scrach end
    
    
    
    
    
    
    
    
    % Plot
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    set(h,'CData',pic);
    %     set(h,'CData',pic_h);
    set(h_title,'String',sprintf('Frame number:%d',frame_num))
    
    %   set(h7,'visible','on','xdata',door_location_1(1),'ydata',door_location_1(2))
    %   set(h8,'visible','on','xdata',door_location_2(1),'ydata',door_location_2(2))
    set(h,'CData',pic);
    %     set(h,'CData',pic_h);
    set(h_title,'String',sprintf('Frame number:%d',frame_num))
    
    %   set(h7,'visible','on','xdata',door_location_1(1),'ydata',door_location_1(2))
    %   set(h8,'visible','on','xdata',door_location_2(1),'ydata',door_location_2(2))
    
    if Appear_red;
        set(h1,'visible','on','xdata',centroids_red(:,1),'ydata',centroids_red(:,2));
    else
        set(h1,'visible','off')
    end
    if Appear_green;
        set(h2,'visible','on','xdata',centroids_green(:,1),'ydata',centroids_green(:,2));
    else
        set(h2,'visible','off')
    end
    if Appear_blue;
        set(h3,'visible','on','xdata',centroids_blue(:,1),'ydata',centroids_blue(:,2));
    else
        set(h3,'visible','off')
    end
    if Appear_yellow;
        set(h11,'visible','on','xdata',centroids_yellow(:,1),'ydata',centroids_yellow(:,2));
    else
        set(h11,'visible','off')
    end
    if Appear_purple;
        set(h12,'visible','on','xdata',centroids_purple(:,1),'ydata',centroids_purple(:,2));
    else
        set(h12,'visible','off')
    end
    if Appear_pause;
        set(h_pause,'visible','on','xdata',centroids_pause(:,1),'ydata',centroids_pause(:,2));
        
    else
        set(h_pause,'visible','off')
    end
    
    if Appear_red && Appear_green; %
        set(h4,'visible','on','xdata',[p_car1_rear(1,:); p_car1_front(1,:)],'ydata',[p_car1_rear(2,:); p_car1_front(2,:)]);
        set(h5,'visible','on','xdata',centroid_car_1(1,:),'ydata',centroid_car_1(2,:));
        %car1
        set(h_text1,'visible','on','position',[centroid_car_1.' 0],'string',{sprintf('Angle:%f',ang_car_1);sprintf('Length:%f',length_car_1)})
        %car2
        set(h_text5,'visible','on','position',[centroid_car_2.' 0],'string',{sprintf('Angle:%f',ang_car_2);sprintf('Length:%f',length_car_2)});
        
        set(h_text7,'visible','on','position',[(centroid_car_2.' + centroid_car_1.')/2 0],'string',{sprintf('Angle:%f',ang_car);sprintf('Length:%f',cardistance)});
        %
        if Appear_blue; %
            %car1
            set(h6,'visible','on','xdata',[centroid_car_1(1,:); p_ball(1,:)],'ydata',[centroid_car_1(2,:); p_ball(2,:)]);
            set(h_text2,'visible','on','position',[(centroid_car_1.' + p_ball.')/2 0],'string',{sprintf('Angle:%f',ang_rel_ball_1);sprintf('Distance:%f',distance_ball_1)});
            %door
            set(h7,'visible','on','xdata',[door_location_1(1),centroid_car_1(1,:)],'ydata',[door_location_1(2);centroid_car_1(2,:)]);
            set(h_text3,'visible','on','position',[(door_location_1 + centroid_car_1.')/2 0],'string',{sprintf('Angle:%f',ang_rel_ball_2);sprintf('Distance:%f',distance_door_2)});
            set(h8,'visible','on','xdata',[door_location_2(1),centroid_car_1(1,:)],'ydata',[door_location_2(2); centroid_car_1(2,:)]);
            set(h_text4,'visible','on','position',[(door_location_2 + centroid_car_1.')/2 0],'string',{sprintf('Angle:%f',ang_rel_ball_3);sprintf('Distance:%f',distance_door_3)});
            set(h9,'visible','on','xdata',[door_location_1(1);centroid_car_1(1,:)],'ydata',[door_location_1(2); centroid_car_1(2,:)]);
            set(h10,'visible','on','xdata',[door_location_2(1); centroid_car_1(1,:)],'ydata',[door_location_2(2); centroid_car_1(2,:)]);
            
            %car2
            set(h13,'visible','on','xdata',[centroid_car_2(1,:); p_ball(1,:)],'ydata',[centroid_car_2(2,:); p_ball(2,:)]);
            set(h_text6,'visible','on','position',[(centroid_car_2.' + p_ball.')/2 0],'string',{sprintf('Angle:%f',ang_rel_ball_5);sprintf('Distance:%f',distance_ball_2)});
            
        else
            set(h6,'visible','on')
            set(h_text2,'visible','off')
            set(h7,'visible','on')
            set(h_text3,'visible','off')
            set(h_text4,'visible','off')
            set(h8,'visible','on')
            set(h9,'visible','on')
            set(h10,'visible','on')
            set(h11,'visible','on')
            set(h12,'visible','on')
        end
    else
        set(h4,'visible','off')
        set(h5,'visible','off')
        set(h_text1,'visible','off')
        set(h_text2,'visible','off')
        set(h6,'visible','off')
        set(h7,'visible','on','xdata',[door_location_1(1),dir_car_1(1)],'ydata',[door_location_1(2); dir_car_1(2)])
        %set(h_text3,'visible','on','position',[(door_location_1 + p_ball.')/2 0],'string',{sprintf('Angle:%f',ang_rel_ball_2);sprintf('\nDistance:%f',dir_door_1)});
        set(h8,'visible','on','xdata',[door_location_2(1),dir_car_1(1)],'ydata',[door_location_2(2); dir_car_1(2)]);
        %set(h_text4,'visible','on','position',[(door_location_2 + p_ball.')/2 0],'string',{sprintf('Angle:%f',ang_rel_ball_3);sprintf('\nDistance:%f',dir_door_2)});
        %set(h9,'visible','on','xdata',[door_location_1(1); p_ball(1,:)],'ydata',[door_location_1(2); p_ball(2,:)]);
        %set(h10,'visible','on','xdata',[door_location_2(1); p_ball(1,:)],'ydata',[door_location_2(2); p_ball(2,:)]);
        set(h_text4,'visible','off')
        set(h_text5,'visible','off')
        set(h_text7,'visible','off')
        set(h_text6,'visible','off')
    end
    drawnow
    
    %%
    % Record
    %
    %     video_1(frame_num) = getframe(gca);
    %
    %     elapsedTime = toc;
    %
end

stop(vid);
delete(vid);
clear vid;

fclose(BT);
BTstatus = BT.status
delete(BT);


fclose(BT);


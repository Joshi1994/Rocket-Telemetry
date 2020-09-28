clear all;
close all;
%delete(instrfind);
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end
fprintf('Started \n Serial Port Opening \n')
s = serial('COM4','BaudRate',115200);
fopen(s);
fprintf('Serial port opened \n')

t = 0;
 

%------------------------------------------------------------------------------------
x = 0;
y = 0;
z = 0;

h2 = plot(x,'Color',[1 0 0]); hold on;
h3 = plot(y,'Color',[1 0 0]);
h4 = plot(z,'Color',[1 0 0]); hold off;
h5 = gca;
xlabel('time (samples)','FontSize',12);
ylim([-10 100]);
set(gca,'YGrid','on');
ylabel('velocity, temp , altitude','FontSize',12);
%title('STM32F4-LIS3DSH (RGB = XYZ)');



N = 100;	% horizontal scale span
t = 1;
k = [1 N];
xlim(k);

%-------------------------------------------------------------------------------------
%while(time < 1e6)
    
%for v = 0:1:10   
 %   fprintf('Enterted While \n')
  %  c=fscanf(s);
   
   % X=regexp(c,' ','split');
   
    %vel = X{1};
    %vel = str2double(vel);
    %x_angle = X{2};
    %x_angle = str2double(x_angle);
    %y_angle = X{3};
    %y_angle = str2double(y_angle);
    %z_angle = X{4};
    %z_angle = str2double(z_angle);
   
    
    
    %GyrY = X{5};
    %GyrY = str2double(GyrY);
    %GyrZ = X{6};
    %GyrZ = str2double(GyrZ);
    %MagX = X{7};
    %MagX = str2double(MagX);
    %MagY = X{8};
    %MagY = str2double(MagY);
    %MagZ = X{6};
    %MagZ = str2double(MagZ);
    
    
    %--------------------------------------------------------------------------
    
    %%%%% let's star the rotation cube%%%%%%%%%%

%%% Initialized the cube

xc=0; yc=0; zc=0;    % coordinated of the center
L=3;                 % cube size (length of an edge)
alpha=1;             % transparency (max=1=opaque)

X = [0 0 0 0 0 1; 1 0 1 1 1 1; 1 0 1 1 1 1; 0 0 0 0 0 1];
Y = [0 0 0 0 1 0; 0 1 0 0 1 1; 0 1 1 1 1 1; 0 0 1 1 1 0];
Z = [0 0 1 0 0 0; 0 0 1 0 0 0; 1 1 1 0 1 1; 1 1 1 0 1 1];

C= [0.1 0.5 0.9 0.9 0.1 0.5];   % color/face

X = L*(X-0.5) + xc;
Y = L/1.5*(Y-0.5) + yc;
Z = L/3*(Z-0.5) + zc;
V=[reshape(X,1,24); reshape(Y,1,24); reshape(Z,1,24)]; %rashape takesthe element of X and it fix them in only one coulomn (in this case)


tic; %to count the seconds


%simulation_duration = 120;

%while(toc<simulation_duration) %stop after "simulation duration" seconds


while(t < 1e6)        
    %a = toc
    
    %---------------------------
    %read serial
    
    fprintf('Enterted While \n')
    c=fscanf(s);
   
    X=regexp(c,' ','split');
   
    vel = X{1};
    vel = str2double(vel)
    x_angle = X{2};
    x_angle = str2double(x_angle);
    y_angle = X{3};
    y_angle = str2double(y_angle);
    z_angle = X{4};
    z_angle = str2double(z_angle);
    heading = X{5};
    heading = str2double(heading);
    temperature = X{6};
    temperature = str2double(temperature);
    pressure = X{7};
    pressure = str2double(pressure);
    altitude = X{8};
    altitude = str2double(altitude);
     
    angle_x = x_angle*pi/180
    angle_y = y_angle*pi/180
    angle_z = z_angle*pi/180
    
%     psi=accel_angle_x;
%     tetha=accel_angle_y;
%     phi=accel_angle_z;
%     yaw=psi;
%     pitch=tetha;
%     roll=phi;
%      x=[-0.5 0.5 0.5 -0.5 -0.5 -0.5 0.5 0.5 -0.5 -0.5 0.5 0.5 0.5 0.5 -0.5 -0.5; -1 -1 1 1 -1 -1 -1 1 1 -1 -1 -1 1 1 1 1;-2 -2 -2 -2 -2 2 2 2 2 2 2 -2 -2 2 2 -2];
%     xn1=[cos(accel_angle_x) 0 sin(accel_angle_x) ; 0 1 0;-sin(accel_angle_x) 0 cos(accel_angle_x)]*x; %rotazione asse x
%     xn2=[1 0 0; 0 cos(accel_angle_y) -sin(accel_angle_y); 0 sin(accel_angle_y) cos(accel_angle_y)]*xn1;
    
    
%     dcm_acc = angle2dcm( roll, pitch, yaw ) %it creates the rotation matrix [angoli di eulero -> (z,y,x)]
%     dcm_gyr = angle2dcm( unfiltered_gyro_angle_z, unfiltered_gyro_angle_y, unfiltered_gyro_angle_x) %it creates the rotation matrix [angoli di eulero -> (z,y,x)]
    dcm_filtered = angle2dcm( angle_z, angle_x, angle_y); %it creates the rotation matrix [angoli di eulero -> (z,y,x)]
% 
%     VR_acc=dcm_acc*V;
%     VR_gyr=dcm_gyr*V;
    VR_filtered=dcm_filtered*V;

%     XR_acc=reshape(VR_acc(1,:),4,6);
%     YR_acc=reshape(VR_acc(2,:),4,6);
%     ZR_acc=reshape(VR_acc(3,:),4,6);
%     
%     XR_gyr=reshape(VR_gyr(1,:),4,6);
%     YR_gyr=reshape(VR_gyr(2,:),4,6);
%     ZR_gyr=reshape(VR_gyr(3,:),4,6);
    
    XR_filtered=reshape(VR_filtered(1,:),4,6);
    YR_filtered=reshape(VR_filtered(2,:),4,6);
    ZR_filtered=reshape(VR_filtered(3,:),4,6);
%     xn=dcm*x;

    
   % figure(1)
%     subplot(1,3,1)
%     fill3(XR_acc,YR_acc,ZR_acc,C,'FaceAlpha',alpha);
%     xlim([-2 2]);
%     ylim([-2 2]);
%     zlim([-2 2]);
%     box on;
%     drawnow
%     subplot(1,3,2)
% 
%     fill3(XR_gyr,YR_gyr,ZR_gyr,C,'FaceAlpha',alpha);
%     xlim([-2 2]);
%     ylim([-2 2]);
%     zlim([-2 2]);
%     box on;
%     drawnow
%     subplot(1,3,3)
    
    figure(2)
    fill3(XR_filtered,YR_filtered,ZR_filtered,C,'FaceAlpha',alpha);
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
    box on; 
    
        
    
    
        velx = vel
		tempy = temperature
		altz = altitude

		%set(h1,'YData',[x y z]);
	
		set(h2,'YData',[get(h2,'YData') velx]);
		%figure(2)
        set(h3,'YData',[get(h3,'YData') tempy]);
		%figure(3)
        set(h4,'YData',[get(h4,'YData') altz]);
		
		t = t+1;
		if(t>N)
			k = k+1;
			set(h5,'XLim',k);
		end
		
    
    
       
    drawnow
    
 
    
%     subplot(2,1,2)
% % 
%     plot3(xn(1,:),xn(2,:),xn(3,:))
%     axis([-3, 3, -3, 3, -3, 3])
%     drawnow
end

    
   




    
    
%end

delete(instrfind);



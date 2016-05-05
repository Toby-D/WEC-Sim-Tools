%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright 2016 Toby Dewhurst
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Animates WEC-Sim simulation currently in the workspace. Saves to video
% file (in "Movie" directory) when mov_save=1. 

% Directions: 
% 1. Put Animator.m on your MATLAB path.
% 2. Add a line in userDefinedFunctions.m that calls "Animator"
% 3. Set values for the following variables. Otherwise, defaults will be
% used.
% 4. Turn on/off visualization of each force on body in Animator.m. 
% NOTE: Colors of force vectors should match default legend for output.plotForces

%% Variables (Set in userDefinedFunctions.m; Defaults set in Animator.m)
% mov_startT=60; %Simulation time at which to start animation. Default: Ramp time
% mov_endT=70.5;   %Simulation time at which to end animation. Default: End
% mov_save=1; %1 to save video. 
% mov_frate=30;
% mov_2D=0;      %0-3D Perspective. 1-2D View
%%
movname=['H=' num2str(waves.H) 'm' ', T=' num2str(waves.T) 's']
resdir='Movie'; %Directory for exporting video
%% Set movie start and end points
if exist('mov_startT','var')
    mov_startkk=find(simu.time>(mov_startT),1,'first');
elseif waves.typeNum<2 %No waves
    mov_startkk=1;
else
    mov_startkk=find(simu.time>(simu.rampT),1,'first');
end

if exist('mov_endT','var')
    mov_endkk=find(simu.time>(mov_endT),1,'first');
else
    mov_endkk=length(simu.time);
end

if ~exist('mov_2D','var')
    mov_2D=0;
end

%% Process body geometry if it hasn't already happened

if (simu.nlHydro == 0) && (simu.paraview == 0) %If body geometry hasn't been processed yet
    for kk = 1:length(body(1,:))
        body(kk).bodyGeo(body(kk).geometryFile)
    end; clear kk
end

%% Set up the figure and view.
figure('units','normalized','outerposition',[0.1 0.1 0.8 0.8])

%Spatial range
x = linspace(-simu.domainSize/2, simu.domainSize/2, waves.viz.numPointsX);
y = linspace(-simu.domainSize/2, simu.domainSize/2, waves.viz.numPointsY);
[X,Y] = meshgrid(x,y);
%Overdeveloped auto-scale module for z-axis
zmin=0; zmax=0;
for ii=1:simu.numWecBodies
zmini=1.2*(min(body(ii).bodyGeometry.vertex(:)+min(output.bodies(ii).position(:,3))-waves.H));
zmaxi=1.2*(max(body(ii).bodyGeometry.vertex(:)+max(output.bodies(ii).position(:,3))+waves.H));
zmin=min(zmin,zmini);
zmax=max(zmax,zmaxi);
end
zlim([zmin zmax])
xlim([-simu.domainSize/2 simu.domainSize/2]);
ylim([-simu.domainSize/2 simu.domainSize/2]);

axis equal
ax=gca;
if mov_2D
    ax.View=[0 0];
else
ax.Projection='perspective';
cam_angle=5; %degrees
cam_distance=(zmax-zmin)/tand(cam_angle);
[xa,ya,za]=sph2cart(-105*pi/180,15*pi/180,cam_distance);
ax.CameraPosition=[xa,ya,za];
cam_angle=5; %degrees
cam_distance=(zmax-zmin)/tand(cam_angle);
[xa,ya,za]=sph2cart(-105*pi/180,15*pi/180,cam_distance);
ax.CameraPosition=[xa,ya,za];
end
ax.CameraTarget=[0 0 0];
% cam_angle=ax.CameraViewAngle;

%% Movie properties
if ~exist('mov_frate','var')
mov_frate= 30; %Overwritten later if dT is too large
end 
if ~exist('mov_save','var')
    mov_save=1; %Save by defualt
end

ftstep=(simu.time(end)-simu.time(mov_startkk))/(length(simu.time)-mov_startkk);
mov_frate=min(mov_frate,1/ftstep); % How many frames per second.
twant_mov=simu.time(mov_startkk):1/mov_frate:simu.time(mov_endkk);
% [N_mov,edges,bin]=histcounts(simu.time,[0 twant_mov]);
[N_mov,~,bin]=histcounts(simu.time,[0 twant_mov]); %Accomodate for adaptive time step solver
its=find(diff(bin)>0);
if mov_save
    if ~exist(resdir, 'dir'); mkdir(resdir); end
    writerObj = VideoWriter([resdir filesep movname '.avi']); % Name it.
    writerObj.FrameRate=mov_frate;
    open(writerObj);
end

% Calculate ramp function
t=waves.waveAmpTime(:,1);
ramp=ones(size(t));
ramp_endkk=find(simu.time>(simu.rampT),1,'first');
ramp(1:ramp_endkk)=1/2*(1+cos(pi+pi*t(1:ramp_endkk)/simu.rampT));

%% Get Forces
%SET f_on TO ZERO FOR FORCES THAT YOU DON'T WANT TO DISPLAY
forcenames{1}='forceTotal';                      f_on(1)=0;
forcenames{2}='forceExcitation';                 f_on(2)=1;
forcenames{3}='forceRadiationDamping';           f_on(3)=0;
forcenames{4}='forceAddedMass';                  f_on(4)=0;
forcenames{5}='forceRestoring';                  f_on(5)=1;
forcenames{6}='forceMorrisonAndViscous';         f_on(6)=0;
forcenames{7}='forceLinearDamping';              f_on(7)=0;
forcenames{8}='forceMooring';                    f_on(8)=0;
fsign(1:2)=1; fsign(3:8)=-1; %Correct for sign
fnkks=1:8;
%Colors for plotting:
clrs=repmat(get(0,'DefaultAxesColorOrder'),2,1);
% Find max forces (for normalizing arrow size)
for ii=1:simu.numWecBodies
    for fnkk=fnkks(logical(f_on))
        fmag_max(fnkk,ii)=max(sqrt(sum((output.bodies(ii).(forcenames{fnkk})(:,1:3).^2),2)));
        mmag_max(fnkk,ii)=max(sqrt(sum((output.bodies(ii).(forcenames{fnkk})(:,4:6).^2),2)));
    end
end
fmag_max=max(fmag_max(:));
mmag_max=max(mmag_max(:));

%Loop for movie
fprintf('Time= ');
msglgth=0; %Setup progress/lagging message
for it=its %mov_startkk:length(t)-1
    tic %count time taken by plotting step
    hold off
    
      % calculate wave elevation
      switch waves.type
          case{'noWave','noWaveCIC','userDefined'}
              Z = zeros(size(X));
          case{'regular','regularCIC'}
              Xt = X*cos(waves.waveDir*pi/180) + Y*sin(waves.waveDir*pi/180);
              Z = ramp(it)*(waves.A * cos(-1 * waves.k * Xt  +  waves.w * t(it)));
          case{'irregular','irregularImport'}
              Z = zeros(size(X));
              Xt = X*cos(waves.waveDir*pi/180) + Y*sin(waves.waveDir*pi/180);
              for iw = 1:length(waves.w)
                  Z = ramp(it)*(Z + sqrt(waves.A(iw)*waves.dw) * cos(-1*waves.k(iw)*Xt + waves.w(iw)*t(it) + waves.phaseRand(iw)));
              end
      end
      
        %Plot wave
        hold on
        surf(X,Y,Z,'LineWidth',0.1,...
            'FaceColor',[0 0 1],'FaceAlpha',0.5,...
               'FaceLighting',    'gouraud',     ...
                'AmbientStrength', 0.15,...
            'EdgeColor',[0.3 0.3 0.3]');

        %Plot body(ies)
        for ii=1:simu.numWecBodies
%             fv(ii).vertices=fv(ii).vertices+output.bodies(ii).position(it,3);

%copied from bodyclass
                %copied from bodyclass
                vertex = body(ii).bodyGeometry.vertex;
                face = body(ii).bodyGeometry.face;
%                 cellareas.ca = body(ii).bodyGeometry.area;
                % calculate new position
                pos=output.bodies(ii).position(it,:); %pos = pos_all(it,:);
                vertex_mod = body.rotateXYZ(vertex,[1 0 0],pos(4));
                vertex_mod = body.rotateXYZ(vertex_mod,[0 1 0],pos(5));
                vertex_mod = body.rotateXYZ(vertex_mod,[0 0 1],pos(6));
                vertex_mod = body.offsetXYZ(vertex_mod,pos(1:3));
                
%                 surf(vertex_mod,'FaceColor',       FaceColors(ii,:), ...

            patch('Faces',face,'Vertices',vertex_mod,'FaceColor',body(ii).viz.color,...
                'FaceAlpha',body(ii).viz.opacity,...
                  'FaceLighting',    'gouraud',     ...
                'AmbientStrength', 0.15);
            
            %% Plot specified forces on body
            if simu.time(it)>=12.3; 
                what=2; 
            end
            
            for fnkk=fnkks(logical(f_on))
%                 fmag_n(fnkk)=sqrt(sum((output.bodies.(forcenames{1})(it,1:3).^2)))/fmagmax; %Force magnitude normalized by max
%                 mmag_n(fnkk)=sqrt(sum((output.bodies.(forcenames{1})(it,4:3).^2)))/mmag_max;%Moment magnitude normalized by max
                dfv(fnkk,:)=output.bodies(ii).(forcenames{fnkk})(it,1:3)/fmag_max; %Force components normalized by max
                dfv(fnkk,:)=fsign(fnkk)*dfv(fnkk,:)*zmax; %Correct for sign of force and scale to fill plot
                pos=output.bodies(ii).position(it,1:3);
                quiver3(pos(1),pos(2),pos(3),pos(1)+dfv(fnkk,1),pos(2)+dfv(fnkk,2),pos(3)+dfv(fnkk,3),'LineWidth',2,'color',clrs(fnkk,:)) %'facealpha',0.5,);
            end

    

% fmag_max=max(fmag_max);
% mmag_max=max(mmag_max);


        end

    %         text(0,yrange-1,titles,'HorizontalAlignment','center','VerticalAlignment','top')
%     xlabel('Horizontal Position, m')

    tstamp(1)={['Time, t=' num2str(round(t(it)*10)/10) ' s']};
    text(-simu.domainSize/2+1,simu.domainSize/2-1,zmax-1,tstamp,'HorizontalAlignment','left','VerticalAlignment','top')
    
    %Display seastate
        param2(1)={['[Significant] Wave Height: ' num2str(waves.H) ' m']};
        param2(2)={['[Peak] Period: ' num2str(waves.T) ' s']};
        text(0,simu.domainSize/2-1,zmax-1,param2,'HorizontalAlignment','left','VerticalAlignment','bottom')

%     
   title([body(1).name])
   
%  ax.CameraPositionMode='auto'; ax.CameraTargetMode='auto'; ax.CameraViewAngleMode='auto'; %(Hack to re-enable stretch-to-fill)
   
if mov_save
   frame = getframe(gcf); % 'gcf' can handle if you zoom in to take a movie.
    writeVideo(writerObj, frame);
end
    pltime=toc; %count time taken by plotting
    
    fprintf(repmat('\b',1,msglgth))
    if pltime>1/mov_frate %Plotting too slow
        msglgth=fprintf('%0.1f s. Lagging! ',simu.time(it));
    else
        msglgth=fprintf('%0.1f s. ',simu.time(it));
%         pause(1/mov_frate-pltime) %Play in real time
    end
    if it==its(1); pause; end; %Pause to allow repositioning of graph
    if it~=its(end); cla; end; %delete htr
end

if mov_save
    close(writerObj); % Saves the movie.
end

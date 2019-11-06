function fig = drawRocket(result, rocket)

% Clear the current figure.
figure(1);
clf;
fig.text.axis = axes('position',[0 0 1 1]);
axis([0 1 0 1]);
hold on
% axis off
grid on
fs = 14;

velocity = norm(result.traj.velI(:,1));
alt = result.traj.posI(3,1);

fig.text.vel = text(0.05,0.975,...
    sprintf('Velocity: %0.2f ',velocity),...
    'fontweight','bold','fontsize',fs,...
    'color','k','verticalalignment','top');
fig.text.alt = text(0.05,0.9,...
    sprintf('Altitude: %0.2f',alt),...
    'fontweight','bold','fontsize',fs,...
    'color','k','verticalalignment','top');

set(gcf,'renderer','opengl');
set(gcf,'color','w');

fig.view0.axis = axes('position',[-0.75 -0.2 2.5 1.5]);
grid on
axis equal;
fig.view0.dx = .5;
fig.view0.dy = .5;
fig.view0.dz = .5;
set(fig.view0.axis,'xlim',[result.traj.posI(1,1)-fig.view0.dx, result.traj.posI(1,1)+fig.view0.dx]);
set(fig.view0.axis,'ylim',[result.traj.posI(2,1)-fig.view0.dy, result.traj.posI(2,1)+fig.view0.dy]);
% set(fig.view0.axis,'zlim',[0-fig.view0.dz, result.traj.posI(3,1)+fig.view0.dz]);
axis manual;
hold on;
axis off;
box on;

view([180-37.5,20]);
set(gca,'projection','perspective');
set(gca,'clipping','on','clippingstyle','3dbox');
lighting gouraud
fig.view0.light = light('position',[-1;1;1],'style','local');

[pointsRocket, facesRocket, cDataRocket, pointsThruster, facesThruster, cDataThruster] = getRocketModel(rocket);

%%
for i = 1:length(result.traj.time)
    velocity = norm(result.traj.velI(:,i));
    if result.traj.velI(3,i) > 1
        velSign = 1;
    else
        velSign = -1;
    end
    alt = result.traj.posI(3,i);
    
    
    set(fig.view0.axis,'xlim',[result.traj.posI(1,i)-fig.view0.dx, result.traj.posI(1,i)+fig.view0.dx]);
    set(fig.view0.axis,'ylim',[result.traj.posI(2,i)-fig.view0.dy, result.traj.posI(2,i)+fig.view0.dy]);
    set(fig.view0.axis,'zlim',[result.traj.posI(3,i)-fig.view0.dz, result.traj.posI(3,i)+fig.view0.dz]);
    
    % - transformations
    pointsRocketI = quatVectorRotation(quatConjugate(result.traj.qi2b(:,1)), pointsRocket);
    pointsRocketI(1,:) = pointsRocketI(1,:) + result.traj.posI(1,i);
    pointsRocketI(2,:) = pointsRocketI(2,:) + result.traj.posI(2,i);
    pointsRocketI(3,:) = pointsRocketI(3,:) + result.traj.posI(3,i);
    
    pointsThruster(3,1) = -norm(result.traj.thrustI(:,i)/100);
    pointsThrusterI = quatVectorRotation(quatConjugate(result.traj.qi2b(:,1)), pointsThruster);
    pointsThrusterI(1,:) = pointsThrusterI(1,:) + result.traj.posI(1,i);
    pointsThrusterI(2,:) = pointsThrusterI(2,:) + result.traj.posI(2,i);
    pointsThrusterI(3,:) = pointsThrusterI(3,:) + result.traj.posI(3,i);
    
    fig.rocket = patch('Vertices',pointsRocketI','Faces',facesRocket,'FaceColor','flat',...
        'FaceVertexCData',cDataRocket,'FaceAlpha',1,'EdgeAlpha',0,...
        'backfacelighting','reverselit','AmbientStrength',0.6);
    
    if norm(result.traj.thrustI(:,i)) ~= 0
        fig.thruster =  patch('Vertices',pointsThrusterI','Faces',facesThruster,'FaceColor','flat',...
        'FaceVertexCData',cDataThruster,'FaceAlpha',1,'EdgeAlpha',0,...
        'backfacelighting','reverselit','AmbientStrength',0.6);
    end
    
    set(fig.text.vel,'string',sprintf('Velocity: %0.2f',velSign*velocity));
    set(fig.text.alt,'string',sprintf('Altitude: %0.2f',alt));
    drawnow;
    pause(0.01)
    
    delete(fig.rocket);
    if isfield(fig,'thruster')
        delete(fig.thruster);
    end
end

end
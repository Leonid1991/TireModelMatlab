function postprocess2( t, y, y0, parameters, dt,st,vid,rel_d,ver_d,rim_d)
Node= parameters{1}(6);
level= parameters{1}(7);
wheel_Rim=parameters{2};
%% Move of the whole object
if vid ==1
    figure();
    for ii=1:floor((1/dt)/100):length(t)
        clf % clear figure (needed to display t correctly)
        present(y(ii,:)',level,Node,wheel_Rim)
        pause(0.1) 
    end
end
%% Relative movement of one point 
if rel_d == 1
    figure();
    for ii=1:10:length(t)
        yy=y(ii,:)';
        % point of interest
        jj=ceil(level/2);  % central level  
        kk=1;  % point 
        if kk==0
            kk=Node;
        end     
        % force_spring2(3,1,2,1,yy,Node,level,cp,l_t,eta,eps)  % forces between 2 points 
        [pos0,~,~]=position_from_y(jj,kk,Node,level,y0);
        [pos,~,~] =position_from_y(jj,kk,Node,level,yy);        
        plot(t(ii),norm(pos-pos0),'.b')
        title('node displ. from initial position')
        ylabel('displacement')
        xlabel('time')
        hold on
        grid on
        grid minor
    end    
end
%% Absolute movement of one point in z axis
if ver_d == 1
    figure();
    for ii=1:floor((1/dt)/300):length(t)
        yy=y(ii,:)';
        % point of interest
        jj=ceil(level/2);  % central level  
        kk=1;  % point 
        if kk==0
            kk=Node;
        end     
        [pos,~,~] =position_from_y(jj,kk,Node,level,yy);        
        plot(t(ii),pos(3),'.k')
        title('node vertical motion')
        ylabel('displacement')
        xlabel('time')
        hold on
        grid on
        grid minor
    end    
end
%% Relative movement of center
if rim_d == 1
    figure();
    for ii=ceil(st/dt):1:length(t)
        yy=y(ii,:)';
        pos =yy(3);
        plot(t(ii),pos,'.k')
        title('rim displ. from initial position')
        ylabel('displacement')
        xlabel('time')
        hold on
        grid on
        grid minor
    end
end    
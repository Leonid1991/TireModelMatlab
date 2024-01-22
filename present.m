function present(yy,level,Node,wheel_Rim,nam_l)     
    hold on
    A = Af(yy(4), yy(5), yy(6),yy(7));
    % rim points
    for kk=1:Node
        [~,~,index1]= position_from_y(1,kk,Node,level,yy); % point data 
        [~,~,index2]= position_from_y(level,kk,Node,level,yy); % point data
        pos1= yy(1:3) + A*wheel_Rim(index1);
        pos2= yy(1:3) + A*wheel_Rim(index2);            
        plot3([pos1(1) pos2(1)],[pos1(2) pos2(2)],[pos1(3) pos2(3)],'-r');
        hold on   

        kl=kk-1;    % points "before (leftside of kk - kl)" the current one within the same layer         
        if kl == 0  % such as it is a wheel, we have a cylce  
            kl=Node;       
        end               
        [~,~,index3]= position_from_y(1,kl,Node,level,yy); % point data 
        [~,~,index4]= position_from_y(level,kl,Node,level,yy); % point data
        pos3= yy(1:3) + A*wheel_Rim(index3);
        pos4= yy(1:3) + A*wheel_Rim(index4);
        plot3([pos3(1) pos1(1)],[pos3(2) pos1(2)],[pos3(3) pos1(3)],'-r');
        plot3([pos4(1) pos2(1)],[pos4(2) pos2(2)],[pos4(3) pos2(3)],'-r');            
    end  
    % nodes markers
    for jj=1:level
        for kk=1:Node
            [pos,~,~]= position_from_y(jj,kk,Node,level,yy); % point data
            if (jj==1) || (jj==level) % wall nodes
                plot3(pos(1),pos(2),pos(3),'ok','MarkerFaceColor', 'b');   
            else % belt nodes
                plot3(pos(1),pos(2),pos(3),'ok','MarkerFaceColor', 'g');
            end   
        end    
    end     

    xlim([-3 3])
    ylim([-1.0 1.0])
    zlim([0 3])
    view([40,0,0]);
    % view([40,80,100]);
    grid on
    hold off
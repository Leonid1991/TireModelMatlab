function [position,velocities,index] = position_from_y(jj,kk,Node,level,y)
         
        index=(jj-1)*(3*Node)+3*(kk-1)+1:(jj-1)*(3*Node)+3*(kk-1)+3;
        
        position  =y(14+index); % +14 are DoFs of Rim 
        velocities=y(14+3*Node*level+index); % +(14+3*Node*level) are DoFs of Rim & positions
        
        
        
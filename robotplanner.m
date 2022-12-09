function[mprim_id] = robotplanner(envmap, res, robotpos, targetpos, mprim);

MEX = 1;

%failed to find an acceptable move
mprim_id = 1;   %arbitrary move

if (MEX == 1)
	%if using MEX, here you would call the planner
    robotpos = robotpos
	mprim_id = planner(envmap, robotpos, targetpos);
    mprim_id = mprim_id + 1;

else
	%otherwise do planning right here 

	%for now greedily move towards the target, 
	%but this is where you can put your planner 
	mindisttotarget = 1000000;
    
    %get direction index for robotpos w.r.t the motion primitives    
    normalized_angle = wrapTo2Pi(robotpos(3));
    dir = fix(normalized_angle / (2*pi / size(mprim, 1)) + 0.5) + 1;
    if (dir == 9)
      dir = 1;
    end;
	for idx = 1:size(mprim, 2)
           [ret, motion] = applyaction(envmap, res, robotpos, mprim, dir, idx);
           new_pos = motion(end,:);
           if (ret == 1)    
               if (new_pos(1) >= 1 & new_pos(1) <= size(envmap, 1) & new_pos(2) >= 1 & new_pos(2) <= size(envmap, 2))         
                    disttotarget = sqrt((new_pos(1)-targetpos(1))^2 + (new_pos(2)-targetpos(2))^2);
                    if(disttotarget < mindisttotarget)
                      mindisttotarget = disttotarget;
                      mprim_id = idx;
                    end;
               end;
           end
	end;
end;

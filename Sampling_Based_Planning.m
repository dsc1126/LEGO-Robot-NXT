function [Waypoints] = Sampling_Based_Planning(position,angle,modifiedMap,target);

%check current position in which area
 if (position(1)>=0 && position(1)<=45) && (position(2)>=59 && position(2)<=106)
 position_area = 1
 end
 if (position(1)>=0 && position(1)<=45) && (position(2)>=45 && position(2)<=59)
 position_area = 2
 end
 if (position(1)>=0 && position(1)<=45) && (position(2)>=0 && position(2)<=45)
 position_area = 3
 end
 if (position(1)>=45 && position(1)<=105) && (position(2)>=59 && position(2)<=106)
 position_area = 4
 end
 if (position(1)>=45 && position(1)<=60) && (position(2)>=0 && position(2)<=45)
 position_area = 5
 end
%check target in which area 
 if (target(1)>=0 && target(1)<=45) && (target(2)>=59 && target(2)<=106)
 target_area = 1
 end
 if (target(1)>=0 && target(1)<=45) && (target(2)>=45 && target(2)<=59)
 target_area = 2
 end
 if (target(1)>=0 && target(1)<=45) && (target(2)>=0 && target(2)<=45)
 target_area = 3
 end
 if (target(1)>=45 && target(1)<=105) && (target(2)>=59 && target(2)<=106)
 target_area = 4
 end
 if (target(1)>=45 && target(1)<=60) && (target(2)>=0 && target(2)<=45)
 target_area = 5
 end
 
 A = [23,82];
 B = [23,23];
 %condition 1: make direct movement
 if ((position_area==1||position_area==4)&&(target_area==1||target_area==4))||((position_area==1||position_area==2||position_area==3)&&(target_area==1||target_area==2||target_area==3))||((position_area==3||position_area==5)&&(target_area==3||target_area==5))
 Waypoints = [position(1),position(2);target(1),target(2)]
 %condition 2: Pass point A only
 elseif (position_area==4&&(target_area==2||target_area==3))||((position_area==2||position_area==3)&&target_area==4)
 Waypoints = [position(1),position(2);A(1),A(2);target(1),target(2)]
 %condition 3: Pass point B only
 elseif (position_area==5&&(target_area==1||target_area==2))||((position_area==1||position_area==2)&&target_area==5)
 Waypoints = [position(1),position(2);B(1),B(2);target(1),target(2)]
 %condition 4: Pass point A then point B
 elseif (position_area==4&&target_area==5)
 Waypoints = [position(1),position(2);A(1),A(2);B(1),B(2);target(1),target(2)]
 %condition 5: Pass point B then point A
 elseif (position_area==5&&target_area==4)
 Waypoints = [position(1),position(2);B(1),B(2);A(1),A(2);target(1),target(2)]
 else
 Waypoints = 0;
 end
 
end
 
 
 
 
 
 
 
 
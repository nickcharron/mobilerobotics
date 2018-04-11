function y = range_bearing_meas_model_exam(xr,map)
% measurement model of range and bearing
% can be found P.12 of Mapping II slides
 if map == 1
     y1 = sqrt((xr(1))^2+(xr(2))^2);
     y2 = 0;
 else
     y1 = 0;
     y2 = atan2(map(2)-xr(2) , map(1)-xr(1)) -xr(3);
 end

 y = [y1;y2];
 
    %y = [ sqrt((xr(1))^2+(xr(2))^2);
    %      atan2(map(2)-xr(2) , map(1)-xr(1)) -xr(3)];
    %y = [sqrt((map(1)-xr(1))^2 + (map(2)-xr(2))^2);
    %     atan2(map(2)-xr(2),map(1)-xr(1))-xr(3)];
end

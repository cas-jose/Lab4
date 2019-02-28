function f = getAngles(short_path_list, point_list) 
%short_path_list is for node list of shortest path
%point_list is for gettingt he coordinates of the nodes of the shortest
%path



for i =1:(length(short_path_list))
    
    
    short_path_vect(i,:) = [point_list(short_path_list(i),1), point_list(short_path_list(i),2)];
    
    %Asssuming robot forward direction is always parallel to y-axis
    
end

for i = 1:(length(short_path_list) - 1)

    vect(i,:) =  [(short_path_vect(i + 1,1)- short_path_vect(i,1)), (short_path_vect(i+1,2) -short_path_vect(i,2))];
    
    if i == 1
        f(i) = -(90 -( atan(vect(i,2)/vect(i,1)) * 180/pi )); %calculating angle of vector
    else
%         e = dot(vect(i,:),vect(i-1,:));
%         d = norm(short_path_vect(i,:));
%         f = norm(short_path_vect(i-1,:));
%         f(i) =  acos(e./(f.*d));
        f(i) = atan2d(vect(i-1,1)*vect(i,2) - vect(i-1,2)*vect(i,1), vect(i-1,1)*vect(i,1) + vect(i-1,2)*vect(i,2));
    end
  
    
    

end


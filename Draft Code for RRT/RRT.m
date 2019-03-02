


k = [0.5, 0.5]; %Start point
lineSize = 0.1; %The length of the line connecting nodes
pointHistory(1,:) = k; %column vector that records all node points
  
 for i =1:99
   q_rx = rand(1); % generating a random node
   q_ry = rand(1);
   
   q = [q_rx, q_ry]; % placing the (x,y) coordinates of the random node in a vector
   
   plot(q_rx,q_ry, "-x"); %plotting the random point ONLY FOR DEBUGGING PURPOSES
   hold on
   
   xlim([-2 2]) %setting the limits of the grid
   ylim([-2 2])
   
   
   if (i == 1) %takes care of first iteration
     
      v = q - k; %gets a vector pointing in the direction from start point K to random node q
  
 
      v_unit = v./norm(v) %creating a unit vector that points in the direction of v
  
      t = v_unit.*lineSize; %scaling the vector with the necessary length "linesize"
      newPoint = k + t; % creates a vector pointing from k to q with length "linesize"
    
   
  
      x = [k(1) newPoint(1)]; %storing the x coordinates of k and newpoint
      y = [k(2) newPoint(2)]; %storing the y coordinates of k and newpoint
      
      scatter(x,y); %plots the points of "x" and "y" as circles on the grid
      hold on %necessary for overlaying the points
      line(x,y) %creates a line connecting k and newpoint using the "x" and "y" vectors
  
  
  %   k = newPoint; might not be needed
      pointHistory(i+1,:) = newPoint; %inserts the new node coordinates into pointHistory
   end
   
   if (i > 1) %takes care of all other 
     
    direction = q - pointHistory; %gets the directional vectors of all node points in "pointHistory" with respect to a new random node q
    
      for m = 1: length(direction) %for loop that calculates the magnitude of each directional vector in "direction" and stores it.
        distance(m,:) = norm(direction(m,:));
      end
      
    [a,b] = min(distance); %after for loop is complete, "a" stores the minimal value in the "distance" vector. "b" stores the index of the minimum value as it appears in the "distance" vector
    %since, "b" is the index of the minimum value and "pointHistory" shares the same indices as "distance", we have now located the nearest node.
   
    v = q - pointHistory(b,:); %gets a vector pointing in the direction from the nearest node to the new random q node
    v_unit = v./norm(v) %creating a unit vector that points in the direction of v
  
    t = v_unit.*lineSize; %scaling the vector with the necessary length "linesize"
    newPoint = pointHistory(b,:) + t; % creates a vector pointing from nearest node to q with length "linesize"
    
   
  
    x = [pointHistory(b,1) newPoint(1)]; %storing the x coordinates of nearest node and newpoint
    y = [pointHistory(b,2) newPoint(2)]; %storing the y coordinates of nearest node and newpoint
    
   scatter(x,y);  %plots the points of "x" and "y" as circles on the grid
   hold on %necessary for overlaying the points
   line(x,y) %creates a line connecting k and newpoint using the "x" and "y" vectors
  
  
 % k = newPoint;
   pointHistory(i+1,:) = newPoint; %inserts the new node coordinates into pointHistory
   end
  
  
  

  
  


 end
  
  


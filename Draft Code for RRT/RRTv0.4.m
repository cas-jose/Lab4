
clear;
clc;

k = [0.5, 0.5]; %Start point
scatter(k(1), k(2),100,'r','filled')
hold on

side = 3; %length of square grid

lineSize = 0.1; %The length of the line connecting nodes
pointHistory(1,:) = k; %column vector that records all node points

%Obstacles-----------------------------------------------------

x_obs = [1, 1,  1.5, 1.5, 1]; 
y_obs = [1, 1.5, 1.5, 1, 1];

plot( x_obs, y_obs, 'b')
hold on

x_obs2 = [2, 2,  2.3, 2, 2]; 
y_obs2 = [1, 1.2, 1.2, 1, 1];

plot( x_obs2, y_obs2, 'b')
hold on


%Creating grid-------------------------------------------------------------------------------
x_grid = [0 0 side side 0]
y_grid = [0 side side 0 0]
plot(x_grid,y_grid, 'b')
axis([-1 4 -1 4])
hold on

%------------------------------------------------------------------------------------------

%Creating the end goal circle------------------------------------------
x_center = 2;
y_center = 1.5;
radius = 0.1;


theta = 0:0.01:2*pi;

x_goal = radius *cos(theta) + x_center;
y_goal = radius *sin(theta) + y_center;

fill(x_goal,y_goal,'g');
hold on
%---------------------------------------------------------------
done = false; % start iteration with assumption that the goal has not been reached in the first iteration

i = 1; %iteration variable
while done == false
    
    
    
    
    if (i == 1) %takes care of first iteration
        
        q_rx = side*rand(1); % generating a random node with constraints with respect to the grid dimensions
        q_ry = side*rand(1);
        
        q = [q_rx, q_ry]; % placing the (x,y) coordinates of the random node in a vector
        
        plot(q_rx,q_ry, "-x"); %plotting the random point ONLY FOR DEBUGGING PURPOSES
        hold on
        
        v = q - k; %gets a vector pointing in the direction from start point K to random node q
        
        
        v_unit = v./norm(v); %creating a unit vector that points in the direction of v
        
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
      Collision = true;
        
        while (Collision == true) %loop that iterates if a new node point is in an obstacle
            q_rx = side*rand(1); % generating a random node with constraints with respect to the grid dimensions
            q_ry = side*rand(1);
            
            q = [q_rx, q_ry]; % placing the (x,y) coordinates of the random node in a vector
            
            plot(q_rx,q_ry, "-x"); %plotting the random point ONLY FOR DEBUGGING PURPOSES
            hold on
            
            
            direction = q - pointHistory; %gets the directional vectors of all node points in "pointHistory" with respect to a new random node q
            
            for m = 1: length(direction) %for loop that calculates the magnitude of each directional vector in "direction" and stores it.
                distance(m,:) = norm(direction(m,:));
            end
            
            [a,b] = min(distance); %after for loop is complete, "a" stores the minimal value in the "distance" vector. "b" stores the index of the minimum value as it appears in the "distance" vector
            %since, "b" is the index of the minimum value and "pointHistory" shares the same indices as "distance", we have now located the nearest node.
            
            v = q - pointHistory(b,:); %gets a vector pointing in the direction from the nearest node to the new random q node
            v_unit = v./norm(v); %creating a unit vector that points in the direction of v
            
            t = v_unit.*lineSize; %scaling the vector with the necessary length "linesize"
            newPoint = pointHistory(b,:) + t; % creates a vector pointing from nearest node to q with length "linesize"
            
            if (inpolygon(newPoint(1,1),newPoint(1,2),x_obs,y_obs) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs2,y_obs2) == false) %conditional statement that checks if a newly created node is inside an obstacle
                Collision = false;
                x = [pointHistory(b,1) newPoint(1)]; %storing the x coordinates of nearest node and newpoint
                y = [pointHistory(b,2) newPoint(2)]; %storing the y coordinates of nearest node and newpoint
                
                scatter(x,y);  %plots the points of "x" and "y" as circles on the grid
                hold on %necessary for overlaying the points
                line(x,y) %creates a line connecting k and newpoint using the "x" and "y" vectors
                pause(0.005) %Used to see the plot in real time
                
                
                % k = newPoint;
                pointHistory(i+1,:) = newPoint; %inserts the new node coordinates into pointHistory
            end
            
            
            
            
        end
        
    end
    
    
    
    
    
    if (inpolygon(pointHistory(i+1,1), pointHistory(i+1,2), x_goal, y_goal)) %checks if the new node lies within the goal
        done = true; %goal is reached, breaks while loop
    end
    
    i = i+1;
    
    
    
    
    
end




clear;
clc;
 
k = [1, 1]; %Start point
scatter(k(1), k(2),100,'r','filled')
hold on

side1 = 24;
side2 = 18; %lengths of rectangle grid

lineSize = 0.5; %The length of the line connecting nodes
pointHistory(1,:) = k; %column vector that records all node points

%Obstacles-----------------------------------------------------

x_obs = [8, 10, 10, 8, 8]; 
y_obs = [18, 18, 12, 12, 18];

x_obs_RU = [10, 10.125, 10, 10]; 
y_obs_RU = [12.4, 12.2, 12, 12.4];

x_obs_RD = [10, 9.8, 9.6, 10];
y_obs_RD = [12, 11.875, 12, 12];

x_obs_LU = [8, 7.875, 8, 8]; 
y_obs_LU = [12, 12.2, 12.4, 12];

x_obs_LD = [8, 8.2, 8.4, 8];
y_obs_LD = [12, 11.875, 12, 12];

%plot( x_obs_RU, y_obs_RU, 'b');
%plot( x_obs_RD, y_obs_RD, 'b');
%plot( x_obs_LU, y_obs_LU, 'b');
%plot( x_obs_LD, y_obs_LD, 'b');

hold on

plot( x_obs, y_obs, 'k')
fill(x_obs,y_obs,'k');

hold on

x_obs2 = [12, 14, 14, 12, 12]; 
y_obs2 = [7, 7, 0, 0, 7];

x_obs2_RU = [13.8, 14, 13.6, 13.8]
y_obs2_RU = [7.125, 7, 7, 7.125]

x_obs2_RD = [14, 14.125, 14, 14]
y_obs2_RD= [7, 6.8, 6.6, 7]

x_obs2_LU = [12.4, 12.2, 12, 12.4]
y_obs2_LU= [7, 7.125, 7, 7]

x_obs2_LD = [12, 11.875, 12, 12]
y_obs2_LD= [7, 6.8, 6.6, 7]

plot( x_obs2, y_obs2, 'k')
fill( x_obs2, y_obs2, 'k')

%plot( x_obs2_RU, y_obs2_RU, 'b');
%plot( x_obs2_RD, y_obs2_RD, 'b');
%plot( x_obs2_LU, y_obs2_LU, 'b');
%plot( x_obs2_LD, y_obs2_LD, 'b');

x_obs3 = [22, 24,  24, 22, 22];
y_obs3 = [3, 3, 0, 0, 3];

x_obs3_LU = [22, 22.2,  22.4, 22];
y_obs3_LU = [3, 3.125, 3, 3];

x_obs3_LD = [22, 22,  21.875, 22];
y_obs3_LD = [3, 2.6, 2.8, 3];

%plot( x_obs3_LU, y_obs3_LU, 'k');
%plot( x_obs3_LD, y_obs3_LD, 'k');

plot( x_obs3, y_obs3, 'k')
fill( x_obs3, y_obs3, 'k')

hold on

x_obs4 = [15, 17, 17, 15, 15]; 
y_obs4 = [18, 18, 15, 15, 18];

x_obs4_RU = [17, 17, 17.125, 17]; 
y_obs4_RU = [15, 15.4, 15.2, 15];

x_obs4_RD = [17, 16.6, 16.8, 17]; 
y_obs4_RD = [15, 15, 14.825, 15];

x_obs4_LU = [15, 15, 14.825, 15]; 
y_obs4_LU = [15, 15.4, 15.2, 15];

x_obs4_LD = [15, 15.4, 15.2, 15]; 
y_obs4_LD = [15, 15, 14.825, 15];

%plot( x_obs4_RU, y_obs4_RU, 'b')
%plot( x_obs4_RD, y_obs4_RD, 'b')
%plot( x_obs4_LU, y_obs4_LU, 'b')
%plot( x_obs4_LD, y_obs4_LD, 'b')

plot( x_obs4, y_obs4, 'k')
fill( x_obs4, y_obs4, 'k')
hold on

x_obs5 = [22, 24,  24, 22, 22]; 
y_obs5 = [18, 18, 15, 15, 18];

x_obs5_LU = [22, 22,  21.875, 22]; 
y_obs5_LU = [15, 15.4, 15.2, 15];

x_obs5_LD = [22, 22.4,  22.2, 22]; 
y_obs5_LD = [15, 15, 14.875, 15];

%plot( x_obs5_LU, y_obs5_LU, 'k')
%plot( x_obs5_LD, y_obs5_LD, 'k')

plot( x_obs5, y_obs5, 'k')
fill( x_obs5, y_obs5, 'k')
hold on
%Creating grid-------------------------------------------------------------------------------
x_grid = [0 0 24 24 0]
y_grid = [0 18 18 0 0]
plot(x_grid,y_grid, 'k')
axis([0 24 0 18])
hold on
grid on
grid minor

%------------------------------------------------------------------------------------------

%Creating the end goal circle------------------------------------------
x_center = 18.5;
y_center = 2;
radius = 0.8;


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
        
        q_rx = side1*rand(1); % generating a random node with constraints with respect to the grid dimensions
        q_ry = side2*rand(1);
        
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
        
        s_node(1,1) = 1; %setting the parent node
        t_node(1,1) = 1;
        s_node(1,2) = 1; %connecting the parent and newly created node
        t_node(1,2) = 2;
        
       
        
        
    end
    
    if (i > 1) %takes care of all other
      Collision = true;
        
        while (Collision == true) %loop that iterates if a new node point is in an obstacle
            q_rx = side1*rand(1); % generating a random node with constraints with respect to the grid dimensions
            q_ry = side2*rand(1);
            
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
            
            if (inpolygon(newPoint(1,1),newPoint(1,2),x_obs,y_obs) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs2,y_obs2) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs3,y_obs3) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs4,y_obs4) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs5,y_obs5) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs_RU,y_obs_RU) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs_RD,y_obs_RD) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs_LU,y_obs_LU) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs_LD,y_obs_LD) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs4_RU,y_obs4_RU) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs4_RD,y_obs4_RD) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs4_LU,y_obs4_LU) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs4_LD,y_obs4_LD) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs2_RU,y_obs2_RU) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs2_RD,y_obs2_RD) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs2_LU,y_obs2_LU) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs2_LD,y_obs2_LD) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs5_LU,y_obs5_LU) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs5_LD,y_obs5_LD) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs3_LU,y_obs3_LU) == false && inpolygon(newPoint(1,1),newPoint(1,2),x_obs3_LD,y_obs3_LD) == false) %conditional statement that checks if a newly created node is inside an obstacle
                Collision = false;
                x = [pointHistory(b,1) newPoint(1)]; %storing the x coordinates of nearest node and newpoint
                y = [pointHistory(b,2) newPoint(2)]; %storing the y coordinates of nearest node and newpoint
                
                scatter(x,y);  %plots the points of "x" and "y" as circles on the grid
                hold on %necessary for overlaying the points
                line(x,y) %creates a line connecting k and newpoint using the "x" and "y" vectors
                pause(0.005) %Used to see the plot in real time
                
                
                % k = newPoint;
                pointHistory(i+1,:) = newPoint; %inserts the new node coordinates into pointHistory
                
                %tracking the nodes and connecting them
                s_node(1, i + 1) = b; %s_node tracks the nearest node
                t_node(1, i + 1) = i + 1; %t_node tracks the newly created node
             
                
            end
            
            
            
            
        end
        
    end
    
    
    
    
    
    if (inpolygon(pointHistory(i+1,1), pointHistory(i+1,2), x_goal, y_goal)) %checks if the new node lies within the goal
        done = true; %goal is reached, breaks while loop
    end
    
    i = i+1;
    
    
    
    
    
end


x_node = transpose(pointHistory(:,1)); %getting the x coordinates of all the nodes for plotting

y_node = transpose(pointHistory(:,2)); %getting the y coordinates of all the nodes for plotting


G = graph(s_node, t_node); %creating a node graph
figure
x_grid = [0 0 24 24 0] %MIGHT BE GOOD TO CREATE A PLOTTING FUNCTION
y_grid = [0 18 18 0 0]
plot(x_grid,y_grid, 'b')
axis([0 24 0 18])
p = plot(G, 'XData',x_node, 'YData', y_node)%plotting the node graph and getting the shortest path
[path1,d] = shortestpath(G,1,i);
highlight(p,path1,'EdgeColor','r')
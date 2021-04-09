function drawWater(x, y, z)

    %figure;
    %hold on;
    %set(gca,'View',[-28,35]); % set the azimuth and elevation of the plot

    xmin = min(x);
    xmax = max(x);
    ymin = min(y);
    ymax = max(y);
    zmin = min(z);
    zmax = 0*max(z);
    
    vert = [xmin ymin zmin;
            xmax ymin zmin;
            xmax ymax zmin;
            xmin ymax zmin;
            xmin ymin zmax;
            xmax ymin zmax;
            xmax ymax zmax;
            xmin ymax zmax];
        
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8]; 
    patch('Vertices', vert, 'Faces', fac, 'FaceVertexCData', repmat([0 0.2 1], 6, 1), ...
            'FaceColor', 'flat', 'FaceAlpha', .05, 'EdgeColor', 'b', 'EdgeAlpha', .5)
end
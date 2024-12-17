function coordarray = ang2coordinate(ori,angle)
    beta = ori.beta;
    a=ori.a;
    l = @(ang) 2*a*cos(beta)/cos(ang/2);
    z = @(ang) a*sqrt((sin(beta)^2-sin(ang/2)^2))/cos(ang/2);
    w = @(ang) 2*a*sin(ang/2);
    xd = @(ang) a*cos(ang/2);

    x1 = @(ang) [0 0 0]';
    x2 = @(ang) [l(ang)/2 0 z(ang)]';
    x3 = @(ang) [l(ang) 0 0]';
    x4 = @(ang) [-xd(ang) w(ang)/2 0]';
    x5 = @(ang) [-xd(ang)+l(ang)/2 w(ang)/2 z(ang)]';
    x6 = @(ang) [-xd(ang)+l(ang) w(ang)/2 0]';
    x7 = @(ang) [0 w(ang) 0]';
    x8 = @(ang) [l(ang)/2 w(ang) z(ang)]';
    x9 = @(ang) [l(ang) w(ang) 0]';
    
    for j = 1:ori.nodenum  
    coordarray((j-1)*3+1:j*3,1) = eval(['x', num2str(j), '(angle)']);  % Evaluate x1(ang), x2(ang)...
    end

end
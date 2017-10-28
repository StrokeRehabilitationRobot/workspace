l0=203.2;       % link lengths taken from LinkArm units are
l1=165.1;
l2=203.2;

x0point=0; %For our arm, the tip of link 1 will always be at (0,0,l0)
y0point=0;
z0point=l0;
points = []
count = 0

for q0 = 0:.1:pi
    for q1 = 0:.1:pi
        for q2 = 0:.1:pi
            count = count+1;
            x1point= l1*cos(q1)*cos(q0);      % forward kinematics for first point
            z1point= l0 + l1 * sin(q1); 
            y1point= l1 * sin(q0) * cos(q1); 

            x2point= x1point+l2*cos(q1+q2)*cos(q0);  % forward kinematics for second point, 
            z2point= z1point + l2 * sin(q2 + q1);    % note y1point is not resused
            y2point= (l1 * cos(q1) + l2 * cos(q1 + q2)) * sin(q0); 
            points(count,:) = [x2point,y2point,z2point]; 
        end
    end
end


[q0,q1,q3] = meshgrid(1:0.5:10,1:20,1:20);



subplot(2,1,1);
plot(points(:,1),points(:,2),'.')
xlabel('x(mm)');
ylabel('y(mm)');
title('x-y Workspace');


subplot(2,1,2);
plot(points(:,1),points(:,3),'.')
xlabel('x(mm)');
ylabel('z(mm)');
title('x-z Workspace');
function v = nlconst_simple(z,p)
global index
%NLCONST Summary of this function goes here
%   Detailed explanation goes here
dist  = p(index.dist);

x_v1  = z(index.x_v1);
y_v1  = z(index.y_v1);
x_v2  = p(index.x_v2);
y_v2  = p(index.y_v2);
slack = z(index.slack);

distance_X=(x_v1-x_v2);
distance_Y=(y_v1-y_v2);
squared_distance_array = sqrt(distance_X.^2+distance_Y.^2);

% Constraint
v = -squared_distance_array+dist-slack;
end


function angle = Angle_cos(PL,PC,PR)
    
% Vectors between the points
vector1 = PL - PC;
vector2 = PR - PC;

% Calculate dot product
dotProduct = dot(vector1, vector2);

% Calculate magnitudes
magnitude1 = norm(vector1);
magnitude2 = norm(vector2);

% Calculate cosine of the angle
cosineTheta = dotProduct / (magnitude1 * magnitude2);

% Calculate the angle in radians
angle = acos(cosineTheta);
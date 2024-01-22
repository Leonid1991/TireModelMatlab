function create_File_for_Chrono(matrix,name)
% Replace these values with your actual matrix elements
% Open the file for writing
fileID = fopen(name, 'w');

% Check if the file is open
if fileID == -1
    error('Cannot open file for writing.');
end

% Determine the size of the matrix
[m, n] = size(matrix);

% Write the matrix to the file with commas and curly braces, each row on a new line
for i = 1:m
    fprintf(fileID, '{');
    for j = 1:n
        fprintf(fileID, '%s', char(matrix(i, j)));
        if j < n
            fprintf(fileID, ', ');
        end
    end
    fprintf(fileID, '},\n');
end

% Close the file
fclose(fileID);

disp(['Matrix has been written to ' name]);

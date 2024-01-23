function create_File_for_Chrono(charData, fileName)
    % Save char-type data to a text file
    
    % Check if the input is a character array
    if ~ischar(charData)
        error('Input must be a character array.');
    end
    
    % Check if the file name is provided
    if nargin < 2
        error('File name must be provided.');
    end
    
    % Add the '.txt' extension to the file name if not provided
    if ~endsWith(fileName, '.txt')
        fileName = [fileName, '.txt'];
    end
    
    % Open the file for writing
    fileID = fopen(fileName, 'w');
    
    % Check if the file was opened successfully
    if fileID == -1
        error('Could not open the file for writing.');
    end
    
    % Write the character data to the file
    fprintf(fileID, '%s', charData);
    
    % Close the file
    fclose(fileID);
    
    disp(['Data saved to ', fileName]);
end
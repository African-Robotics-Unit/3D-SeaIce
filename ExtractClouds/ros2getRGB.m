function ros2getRGB(pathname,outputName)
%pathname="C:\Users\agori\OneDrive - University of Cape Town\ROS2 Validation\2blocks\";
files = dir(strcat(pathname, 'rosbag2*'));
files = files([files.isdir]); % Ensure only directories
len = length(files);
%outputName="C:\Users\agori\Documents\MATLAB\MSc\Validation\2blocksROS2\RGB_blocks\";
for i=1:len
    filename = files(i).name;
    fnames{i}=filename;
    bagReader = ros2bagreader(strcat(pathname,filename));
    %bagSelRS=select(bagReader,"Topic","/camera/color/image_rect_raw");
    bagSelRS=select(bagReader,"Topic","/camera/color/image_raw");
    msgs = readMessages(bagSelRS);
    myRSimg=msgs{4,1};
    picTest=rosReadImage(myRSimg);
    imgFilename = fullfile(outputName, sprintf('%s.png', filename));
    imwrite(picTest,imgFilename);
end
end
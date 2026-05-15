function [tforms] = extractALNtransformRigid(alnFilename)
filestr = fileread(alnFilename);
%break it into lines
filebyline = regexp(filestr, '\n', 'split');
%remove empty lines
filebyline( cellfun(@isempty,filebyline) ) = [];
ntforms = filebyline(1);
tforms = cell(ntforms);
tformCount = 0;
for k=1:length(filebyline)
    if contains(filebyline{k},"#")
        tformCount = tformCount + 1;
        row1 = split(filebyline{k+1}," ");
        row1(end)=[];

        row2 = split(filebyline{k+2}," ");
        row2(end)=[];

        row3 = split(filebyline{k+3}," ");
        row3(end)=[];
        
        row4 = split(filebyline{k+4}," ");
        row4(end)=[];
        
        mytform = str2double([row1, row2, row3,row4]);
        mytform = reshape(mytform,[4, 4]).';
        %tforms{tformCount} = affinetform3d(mytform);
        tforms{tformCount}=makeRigid(mytform);
        k=k+5;
    end
end
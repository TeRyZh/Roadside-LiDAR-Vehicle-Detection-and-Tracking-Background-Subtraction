function foreground = rowByRowThrld(img)

[ROW, COL] = size(img);

foreground = zeros(ROW, COL); 

for i = 1 : ROW
    row_IM = img(i,:);
    row_IM_foreground = zeros(size(row_IM)); 
    [lbBackInt, ubBackInt]=thresholding(row_IM);
    
    row_IM_foreground(row_IM <= lbBackInt | row_IM >= ubBackInt) = 255; 
    foreground(i,:) = row_IM_foreground;
end







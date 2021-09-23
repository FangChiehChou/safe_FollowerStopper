



color_bwr = bluewhitered(256);

x = 1:1:size(color_bwr,1);
y = 1:1:size(color_bwr,1);
c = 

temp_scatter_h = scatter(x,y,100,color_bwr,...
    'o','filled','MarkerFaceAlpha',1.0,'MarkerEdgeAlpha',1.0);
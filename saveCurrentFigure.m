function saveCurrentFigure(fileName)
 % open file.fig
  saveas(gcf, fileName, 'epsc2');

end
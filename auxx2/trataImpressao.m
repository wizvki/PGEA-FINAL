function trataImpressao(h, nome, fontsize)
    
    % muda propriedades da figura
    set(findall(h, 'type','text'), 'fontSize', fontsize, 'fontName', 'Times')
    set(findall(h, 'type','axes'), 'fontSize', fontsize, 'fontName', 'Times')
    
    iResolution = 300;
    set(h, 'PaperPositionMode', 'auto');
    set(h, 'PaperOrientation','landscape');
    
    print('-dpdf', sprintf('-r%d', iResolution), strcat(nome, '.pdf'));
end
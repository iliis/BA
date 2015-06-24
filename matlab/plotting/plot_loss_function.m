x = -1:0.01:1;

h = figure;

subplot(1,2,1);
plot(x, huber_loss(x, 0.1));
title('$huber\_loss(x, 0.1)$', 'interpreter', 'latex')

subplot(1,2,2);
plot(x,(sqrt(huber_loss(x, 0.1))./abs(x) .* x).^2);
title('$(\frac{x}{|x|}\sqrt{huber\_loss(x, 0.1)})^2$', 'interpreter', 'latex')


set(h,'Units','centimeters');

pos = get(h,'Position');

set(h,'PaperPositionMode','Auto','PaperUnits','centimeters','PaperSize',[pos(3), pos(4)])

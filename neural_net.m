load('akshay-trajdata.mat', 'X', 'Y')
X_a = X;
Y_a = Y;

load('regan-trajdata.mat', 'X', 'Y')
X_r = X;
Y_r = Y;

X = [X_a X_r];
Y = [Y_a Y_r];

net = fitnet([30, 20]);
% view(net)

[net,tr] = train(net,X,Y);
save('net808_30_20', 'net');
% nntraintool
% 
% plotperform(tr)
% 
% testX = X(:,tr.testInd);
% testY = Y(:,tr.testInd);
% 
%testO = net(testX);
% 
% perf = mse(net,testT,testO)

%%
% Another measure of how well the neural network has fit the data is the
% regression plot.  Here the regression is plotted across all samples.
%
% The regression plot shows the actual network outputs plotted in terms of
% the associated target values.  If the network has learned to fit the data
% well, the linear fit to this output-target relationship should closely
% intersect the bottom-left and top-right corners of the plot.
%
% If this is not the case then further training, or training a network with
% more hidden neurons, would be advisable.

% O = net(X);

% plotregression(Y,O)

%%
% Another third measure of how well the neural network has fit data is the
% error histogram.  This shows how the error sizes are distributed.
% Typically most errors are near zero, with very few errors far from that.

% e = Y - O;
% 
% ploterrhist(e)
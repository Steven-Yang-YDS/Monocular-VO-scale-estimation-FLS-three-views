function color = multiple_boxplot(data,xlab,Mlab,colors)
% data is a cell matrix of MxL where in each element there is a array of N
% length. M is how many data for the same group, L, how many groups.
%
% Optional:
% xlab is a cell array of strings of length L with the names of each
% group
%
% Mlab is a cell array of strings of length M
%
% colors is a Mx4 matrix with normalized RGBA colors for each M.
% check that data is ok.
if ~iscell(data)
    error('Input data is not even a cell array!');
end
% Get sizes
M=size(data,2);
L=size(data,1);
if nargin>=4
    if size(colors,1)~=M
        error('Wrong amount of colors!');
    end
end
if nargin>=2
    if length(xlab)~=L
        error('Wrong amount of X labels given');
    end
end
% Calculate the positions of the boxes
positions=1:0.25:M*L*0.25+1+0.25*L;
positions(1:M+1:end)=[];
% Extract data and label it in the group correctly
x=[];
group=[];
for ii=1:L
    for jj=1:M
        aux=data{ii,jj};
        x=vertcat(x,aux(:));
        group=vertcat(group,ones(size(aux(:)))*jj+(ii-1)*M);
    end
end
% Plot it
boxplot(x,group, 'positions', positions, 'Symbol','','OutlierSize',3);
%boxplot(x,group, 'positions', positions, 'PlotStyle','compact');
 
 
% Set the Xlabels
aux=reshape(positions,M,[]);
labelpos = sum(aux,1)./M;
set(gca,'xtick',labelpos)
if nargin>=2
    set(gca,'xticklabel',xlab);
else
    idx=1:L;
    set(gca,'xticklabel',strsplit(num2str(idx),' '));
end
     
% Get some colors
if nargin>=4
    cmap=colors;
else
    cmap = hsv(M); % jet, hsv
    hsv_array = hsv(2*M);
    cmap = hsv_array((1:M)*2,:);
    %cmap=vertcat(cmap,ones(L,1)*0.5);
    cmap=[cmap,ones(M,1)*0.5];
end
color=repmat(cmap, L, 1);
% Apply colors
h = findobj(gca,'Tag','Box');
for jj=1:length(h)
   patch(get(h(jj),'XData'),get(h(jj),'YData'),color(jj,1:3),'FaceAlpha',color(jj,4));
end
if nargin>=3
    legend(fliplr(Mlab));
end
 
end
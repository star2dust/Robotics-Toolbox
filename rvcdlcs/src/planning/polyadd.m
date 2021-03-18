function [nodez,nodezinpoly,nodegrid,edgez] = polyadd(nodez,nodepoly,nodezinpoly,edgez,randpolynum,config,envir,robotnum,dispbaseref,dispendref)
% Add edges for new nodes in the node set of a polygraph

% take number of polytope
polynum = length(nodepoly);
% add poly to graph one by one
for i=polynum-randpolynum+1:polynum % random ploytope
    for j=1:polynum-randpolynum % exist ploytope
        [viaz,exit] = poly2nodez([nodepoly(i),nodepoly(j)],nodez(nodezinpoly{i}(1),1:2),config,envir,robotnum,dispbaseref,dispendref);
        if exit==1
            nodez = [nodez;viaz]; znum = size(nodez,1);
            for kj=1:length(nodezinpoly{j})
                edgez = [edgez;nodezinpoly{j}(kj),znum;znum,nodezinpoly{j}(kj)];
            end
            nodezinpoly{j} = [nodezinpoly{j},znum];   
            for ki=1:length(nodezinpoly{i})
                edgez = [edgez;nodezinpoly{i}(ki),znum;znum,nodezinpoly{i}(ki)];
            end
            nodezinpoly{i} = [nodezinpoly{i},znum];
        end
    end
end
randpolynum = size(nodez,1);
edgenum = size(edgez,1);
nodegrid=zeros(randpolynum,randpolynum);
for i=1:edgenum
    nodegrid(edgez(i,1),edgez(i,2))=1;
    nodegrid(edgez(i,2),edgez(i,1))=1;
end
%%%%%
% hold on;
% for kk=1:2:size(edgez,1)
%     x1=[nodez(edgez(kk,1),1);nodez(edgez(kk,2),1)];
%     y1=[nodez(edgez(kk,1),2);nodez(edgez(kk,2),2)];
%     line(x1,y1);
% end
% hold off;
%%%%%
end
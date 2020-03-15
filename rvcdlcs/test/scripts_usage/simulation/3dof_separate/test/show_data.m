function [F,V,C] = show_data(d)
k = 0;
for j=1:length(d)
    for i=1:length(d(j).struct)
        k = k+1;
        if d(j).type=="rgb"
            V{k} = [0,d(j).struct(i).l,d(j).struct(i).l,0;
                0,0,d(j).struct(i).w,d(j).struct(i).w]'-[d(j).struct(i).l/2,d(j).struct(i).w/2];
        else
            V{k} = [0,d(j).struct(i).l,d(j).struct(i).l,0;
                0,0,d(j).struct(i).w,d(j).struct(i).w]'-[0,d(j).struct(i).w/2];
        end
        F{k} = [1,2,3,4];
        C{k} = d(j).struct(i).c;
%         h(k) = patch('Faces',F,'Vertices',V,'FaceColor',C,'EdgeColor',C);
        p0 = [V{k}';ones(size(V{k}(:,1)'))];
        if d(j).type=="rgb"
            p1 = d(j).state.T*d(j).struct(i).TC.T*p0;
        else
            if i>1
                d(j).struct(i).TB = d(j).struct(i-1).TE;
            end
            R = [cos(d(j).state(i)),-sin(d(j).state(i));
                sin(d(j).state(i)),cos(d(j).state(i));];  
            p1 = d(j).struct(i).TB.T*SE2(R).T*p0;
            d(j).struct(i).TE = d(j).struct(i).TB*SE2(R)*SE2([d(j).struct(i).l,0]);
        end
        V{k} = p1(1:2,:)';
    end
end
end
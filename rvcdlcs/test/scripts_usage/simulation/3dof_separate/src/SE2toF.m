function F = SE2toF(SE2)
for i=1:length(SE2)
    F(:,i) = [SE2(i).t;vex(logm(SE2(i).R))];
end
end
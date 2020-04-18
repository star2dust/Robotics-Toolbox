function y = proj_square(x,xl,xu)
y = zeros(size(x));
for i=1:length(x)
   if x(i)<xl(i)
       y(i)=xl(i);
   else
       if x(i)>xu(i)
           y(i)=xu(i);
       else
           y(i)=x(i);
       end
   end
end
end
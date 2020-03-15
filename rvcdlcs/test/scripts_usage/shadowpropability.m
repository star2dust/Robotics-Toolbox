p2 = 0.0006; p1 = 0.0003; d = 400; 
% get 0,1,2 ur within 400 packs
n = d*8;
res1 = nchoosek(n,1)*p2*(1-p2)^(n-1); % get 1 ur from 1/400
for i=2:5
    res1 = res1 + 2*nchoosek(n,i)*p1^i*(1-p2)^(n-i); % get the same ur from i/400
end
res0 = (1-p2)^n; % get 0 ur
res2 = 1-res0-res1; % get 2 or more ur

% get a given ur within 400 packs
resg = 1-(1-p1)^n;

% get 2 >= get 1
res2 = 0; res1 = 1;
while res2<=res1
    n = d*8;
    res1 = nchoosek(n,1)*p2*(1-p2)^(n-1);
    for i=2:5
        res1 = res1 + 2*nchoosek(n,i)*p1^i*(1-p2)^(n-i);
    end
    res0 = (1-p2)^n;
    res2 = 1-res0-res1;
    d = d+1;
end

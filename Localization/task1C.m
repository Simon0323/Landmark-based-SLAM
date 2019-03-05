state = ones(3,10)*0.1;
p1 = 0.8;
p2 = 0.4;
index1 = 1;
index2 = 4;
index3 = 7;
for j = 1:10
    if (j == index1 || j == index2 || j == index3)
         state(1,j) = 0.8 * state(1,j);
    else 
         state(1,j) = 0.4 * state(1,j);
    end
end
temp_sum = sum(state(1,:));
state(1,:) = state(1,:)/temp_sum


for j = 1:10
    index = j-3;
    if (index<1)
        index = 10+index;
    end
    if (j == index1 || j == index2 || j == index3)
         state(2,j) = 0.8 * state(1,index);
    else 
         state(2,j) = 0.4 * state(1,index);
    end
end
temp_sum = sum(state(2,:));
state(2,:) = state(2,:)/temp_sum


for j = 1:10
    index = j-4;
    if (index<1)
        index = 10+index;
    end
    if (j == index1 || j == index2 || j == index3)
         state(3,j) = 0.2 * state(2,index);
    else 
         state(3,j) = 0.6 * state(2,index);
    end
end

temp_sum = sum(state(3,:));
state(3,:) = state(3,:)/temp_sum
function [MaxDis] = max_distance(A)

    x = A(:,1)'; % x component of coordinate
    y = A(:,2)'; % y component of coordinate
    count = 1;
    for i = 1:length(x) - 1
        for j = i + 1:length(x)
        distance(count) = sqrt((x(i) - x(j))^2 + (y(i) - y(j))^2);
        Matrix(count, :) = [x(i) y(i) x(j) y(j) distance(count)];
        count = count + 1;
        end
    end
    SortedMatrix = sortrows(Matrix, 5);
    MaxDis = SortedMatrix(size(Matrix, 1), :);
end
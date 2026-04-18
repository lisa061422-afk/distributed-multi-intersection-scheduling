% % 定义 Merge cell 结构
% Merge = {
%     {1, [1 1 1 1]};
%     {2, [1 2 1 1]};
%     {3, [1 1 1 1]};
%     {4, [1 2 1 1]};
%     {5, [2 2 2 2]}
% };
% result = group_same_ni1(Merge)

function result = group_same_ni(Merge)

% 初始化结果 cell 数组和标记数组
result = {};
processed = false(1, length(Merge)); % 标记哪些项已处理

% 遍历 Merge cell 结构
for i = 1:length(Merge)
    if ~processed(i)%没有被处理过
        % 当前项的行向量
        vec_i = Merge{i}{2};
        group = Merge{i}{1};  % 初始化当前组，包含第一个数字
        processed(i) = true;  % 将当前项标记为已处理

        % 查找相同行向量的项
        for j = i+1:length(Merge)
            if ~processed(j) && isequal(Merge{j}{2}, vec_i)
                group = [group, Merge{j}{1}];  % 将数字添加到当前组
                processed(j) = true;           % 标记为已处理
            end
        end

        % 将当前组添加到结果中
        result{end+1} = group;
    end
end

%disp(result);

end

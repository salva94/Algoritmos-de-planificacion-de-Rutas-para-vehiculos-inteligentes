function child_nodes = child_nodes_cal(parent_node, m, n, obs, closeList)

child_nodes = [];
field = [1,1; n,1; n,m; 1,m];

% 1er nodo hijo
child_node = [parent_node(1)-1, parent_node(2)+1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 1do nodo hijo
child_node = [parent_node(1), parent_node(2)+1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 3er nodo hijo
child_node = [parent_node(1)+1, parent_node(2)+1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 4to nodo hijo
child_node = [parent_node(1)-1, parent_node(2)];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 5to nodo hijo
child_node = [parent_node(1)+1, parent_node(2)];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 6to nodo hijo
child_node = [parent_node(1)-1, parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 7mo  nodo hijo
child_node = [parent_node(1), parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

% 8vo nodo hijo
child_node = [parent_node(1)+1, parent_node(2)-1];
if inpolygon(child_node(1), child_node(2), field(:,1), field(:,2))
    if ~ismember(child_node, obs, 'rows')
        child_nodes = [child_nodes; child_node];
    end
end

%% Excluir nodos que ya existen en closeList
delete_idx = [];
for i = 1:size(child_nodes, 1)
    if ismember(child_nodes(i,:), closeList , 'rows')
        delete_idx(end+1,:) = i;
    end
end
child_nodes(delete_idx, :) = [];
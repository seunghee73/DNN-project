%rng('default')
dataset = [];
env = environment();
[direct, cost] = direction();

for i = 1 : 49999
    [starting, destination] = position (env);
    if (starting(1,1) == destination (1,1)) & (starting(1,2) == destination(1,2))
        while 1 
            [starting, destination] = position (env);
            if (starting(1,1) ~= destination (1,1)) || (starting(1,2) ~= destination(1,2))
                break
            end
        end
    end
    heur = heuristic(env, destination);
    openlist = [starting(1,1), starting(1,2), 0, 0];
    closelist = [];
    [distance] = A_star ( starting, destination, env, direct, cost, heur, openlist, closelist);
    [weight, speed] = weight_speed_func();
    [out] = output(speed, distance);
    data = [starting ,destination, distance, weight, out];
    dataset = [dataset; data];
end

csvwrite('dataset50000.csv', dataset)

%% environment
function [env] = environment()
    env = csvread('environment.csv');
end

%% starting, goal position
function [starting, destination ] = position (env)
    starting(1,1) = randi(size(env,1));
    starting(1,2) = randi(size(env,2));
    destination(1,1) = randi(size(env,1));
    destination(1,2) = randi(size(env,2));
    if env(starting(1,1),starting(1,2)) == 1 
        while 1
            starting(1,1) = randi(size(env,1));
            starting(1,2) = randi(size(env,2));
            if env(starting(1,1),starting(1,2)) == 0
                break
            end
        end
    end
    if env(destination(1,1),destination(1,2)) == 1 
        while 1
            destination(1,1) = randi(size(env,1));
            destination(1,2) = randi(size(env,2));
            if env(destination(1,1),destination(1,2)) == 0
                break
            end
        end
    end
end

%% heuristic
function [heur] = heuristic (env, destination)
    heur = zeros(size(env));
    for i = 1 : size(env,1) 
        for j = 1 : size (env,2)
            heur(i,j) = abs ( destination (1,1) - i ) + abs ( destination (1,2) - j );
        end
    end
end

%% direction
function [direct, cost] = direction()
    direct = [ 0,1; 0,-1; 1,0; -1,0; -1,1; 1,1; -1,-1; 1,-1];
    cost = [ 1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2) ];
    % 위 : 1, 아래 : 2, 오른쪽 : 3, 왼쪽 : 4, 좌상향 : 5, 우상향 : 6, 좌하향 : 7, 우하향 : 8
end

%% list
function [bool] = found(node, list)
    bool = false;
    for j = 1 : size(list,1)
        if (node(1,1) == list(j,1)) && (node(1,2) == list(j,2)) 
            bool = true;
        end
    end
end

%%
function [openlist, closelist, present_node, MOVE, END] = loop(openlist, closelist, present_node, MOVE, END, direct, cost, env, min, heur)    
    next_node = present_node;
    closelist = [closelist; openlist(1,:)];
    for i = 1 : size(cost,2)
        MOVE = false;
        ADD = false;
        new_node = [ present_node(1,1), present_node(1,2)] + direct(i,:);
        
        if (new_node(1,1) > 0) & (new_node(1,1) <= size(env,1)) & (new_node(1,2) > 0 ) & (new_node(1,2) <= size(env,2))  & (found(new_node, closelist) == false) 
            if (env(new_node(1,1),new_node(1,2)) == 0)
                g = openlist(1,3) + cost(i);
                f = g + heur(new_node(1,1), new_node(1,2));

                for p = 1 : size(openlist,1)
                    if (openlist(p,1) == new_node(1,1)) & (openlist(p,2) == new_node(1,2))
                        if openlist(p,3) > g
                            openlist(p,3) = g;
                            openlist(p,4) = f;
                        end
                        ADD = true;
                    end
                end
                if ADD == false 
                    openlist = [openlist ; new_node(1,1), new_node(1,2), g, f ];
                end
                if f < min 
                    min = f;
                    next_node = new_node;
                    MOVE = true;
                end
            end
        end
    end
    present_node = next_node;
    openlist = sortrows(openlist(2:end,:), 4);
    if MOVE == false
        present_node = [openlist(1,1), openlist(1,2)];
        MOVE = true;
    end
end

%% a-star
function [distance] = A_star ( present_node, destination, env, direct, cost, heur, openlist, closelist)
    next_node = present_node;
    min = size(env,1) * size (env, 2);
    closelist = openlist(1,:);
    MOVE = false;
    END = false;
    for i = 1 : size(cost,2)
        new_node = [ present_node(1,1), present_node(1,2)] + direct(i,:);
        if (new_node(1,1) > 0) & (new_node(1,1) <= size(env,1)) & (new_node(1,2) > 0 ) & (new_node(1,2) <= size(env,2)) 
            if (env(new_node(1,1),new_node(1,2)) == 0)
                g = cost(i);
                f = g + heur(new_node(1,1), new_node(1,2));
                openlist = [openlist ; new_node(1,1), new_node(1,2), g, f ];
                if f < min 
                    min = f;
                    next_node = new_node;
                end
            end
        end
    end
    present_node = next_node;
    openlist = sortrows(openlist(2:end,:), 4);
    if destination == present_node
       END = true;
       distance = openlist(1,3);
    end
    while (END == false)
        [openlist, closelist, present_node, MOVE, END] = loop(openlist, closelist, present_node, MOVE, END, direct, cost, env, min, heur);
        if MOVE == false 
           print = 'Search fail';
        end
        if (destination(1,1) == present_node(1,1)) & (destination(1,2) == present_node(1,2))
           END = true;
           distance = openlist(1,3);
        end
    end
end

%% weight, speed
function [weight, speed] = weight_speed_func()
    weight = 100 * rand();
    speed = - 0.045 * weight + 6;
end

%% calculate output
function [out] = output(speed, distance)
    out = distance / speed;
end

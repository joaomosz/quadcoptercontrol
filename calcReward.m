function [r] = calcReward(X_set,X)
%this function calculates the reward used in the RL optimization
    error = X-X_set;
    r = 1/(norm(error)+1);
end


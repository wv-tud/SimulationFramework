function [ nextGenomes ] = mutateGenome( nr, bestGenomes, mutationP, noiseLevel)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    nextGenomes = zeros(nr,13);
    nrOffspring = (nr-size(bestGenomes,1))/size(bestGenomes,1);
    parent = 1;
    for i=1:nr
        if i <= size(bestGenomes,1)
            nextGenomes(i,:) = bestGenomes(i,:);
        else
            nextGenomes(i,:) = unifrnd(-noiseLevel,noiseLevel,1,length(bestGenomes(parent,:))) + bestGenomes(parent,:) .* (1 + mutationP * randn(1,length(bestGenomes(parent,:))));
            if mod((i-1) - length(bestGenomes),nrOffspring) == 0
                parent = parent + 1;
            end
        end
    end
end


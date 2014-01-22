function [c] = quaternionInv(a)
   
    c = quaternionConj(a) / (norm(a)^2);
    
end
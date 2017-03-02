function [c, ceq] = nonLinCon(z)
    % Need variables defined in main file
    global N mx lambda_t alpha beta;
    c = zeros(N,1);
    ceq = [];
    for i=1:N
        lambda_k = z(1+(i-1)*mx);
        e_k = z(5+(i-1)*mx);
        c(i) = alpha*exp(-beta*(lambda_k-lambda_t)^2) - e_k;
    end
end
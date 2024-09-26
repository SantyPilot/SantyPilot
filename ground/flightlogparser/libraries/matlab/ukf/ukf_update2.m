
function [M,P,K] = ukf_update2(M,P,Y,R,h_param,alpha,beta,kappa,mat)

 
  %
  % Do transform and make the update
  %
  tr_param = {alpha beta kappa mat};
  [MU,S,C] = ut_transform1(M,P,h_param,tr_param);
  
  S = S + R;
  K = C / S;
  M = M + K * (Y - MU);
  P = P - K * S * K';
  
  if nargout > 5
    LH = gauss_pdf(Y,MU,S);
  end
#ifndef _UKF_UPDATE2_MAIN_H
#define _UKF_UPDATE2_MAIN_H

extern void main_ukf_update2();
// ukf_update2(Xpre, Ppre, raIn(:,i), R, hparam, alpha, beta, kappa, mat);
extern void main_ukf_update2(std::vector<double>& Xpre,
    std::vector<std::vector<double>>& Ppre,
    std::vector<double>& Z,
    std::vector<double>& Rdiag,
    std::vector<double>& h_param,
    double alpha,
    double beta,
    double kappa,
    double mat,
    std::vector<double>& X,
    std::vector<std::vector<double>>& P);

#endif

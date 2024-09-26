#include "ukf_predict2/main.h"
#include "ukf_track_h_ins/main.h"
#include "ukf_update2/main.h"
#include <iostream>

int main() {
    std::vector<double> X = { -0.0129, -0.0624, -0.1777, 0.8182,
        -0.1515, -0.1924, 0.9999, -0.0056, 0.0060, -0.0082, 1.5503e-04,
        -2.2709e-05, 2.9835e-04};

    std::vector<double> diagP = {25.0, 25.0, 25.0, 5.0, 5.0, 5.0,
        1e-4, 1e-4, 1e-4, 1e-4, 1e-6, 1e-6, 1e-6};
    auto diagQ = diagP;

    std::vector<double> f_param = {0.0053, -0.4103, -1.4706,
        0.8612, 0.1112, 0.0608, -9.8812};
    double alpha = 0.1;
    double beta = 2;
    double kappa = 0;
    double mat = 1;

    std::vector<std::vector<double>> Ppre;
    std::vector<double> Xpre;

    main_ukf_predict2(X, diagP, diagQ, f_param,
        alpha, beta, kappa, mat, Xpre, Ppre);

    /*
    std::cout << "print P:\n";
    for (auto i = 0; i < 13; i++) {
        for (auto j = 0; j < 13; j++) {
            std::cout << Ppre[i][j] << " ";
        } 
        std::cout << std::endl;
    }
    std::cout << "print X:\n";
    for (auto i = 0; i < X.size(); i++) {
        std::cout << Xpre[i];
    }
    std::cout << std::endl;
    */
    // main_ukf_predict2();


    std::vector<double> h_param = {26000, 400, 40000};
    std::vector<double> Zpre;
    main_ukf_track_h_ins(Xpre, h_param, Zpre);
    /*
    std::cout << "print Zpre:\n";
    for (auto i = 0; i < Zpre.size(); i++) {
        std::cout << Zpre[i];
    }
    std::cout << std::endl;
    */
    std::vector<double> Z = {
        -0.0675171000000000,
        0.213371000000000,
        -0.111419000000000,
        0.108268000000000,
        0.678770000000000,
        -0.0561367000000000,
        26000,
        400,
        40000,
        0 };
    std::vector<double> Rdiag = {
        0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 
        10, 10, 10, 0.01
    };
    
    std::vector<double> Xcor;
    std::vector<std::vector<double>> Pcor;
    main_ukf_update2(Xpre,
            Ppre,
            Z,
            Rdiag,
            h_param,
            alpha,
            beta,
            kappa,
            mat,
            Xcor,
            Pcor);

    /*
    std::cout << "print X\n";
    for (auto i = 0; i < Xcor.size(); i++) {
        std::cout << Xcor[i] << " ";
    }
    std::cout << "\n";
    std::cout << "print P\n";
    for (auto i = 0; i < Pcor.size(); i++) {
        for (auto j = 0; j < Pcor[i].size(); j++) {
            std::cout << Pcor[i][j] << " ";
        }
        std::cout << "\n";
    }
    std::cout << std::endl;
    */
    return 0;
}

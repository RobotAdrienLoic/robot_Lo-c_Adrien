#ifndef ASSERVISSEMENT_H
#define	ASSERVISSEMENT_H

typedef struct _PidCorrector
{
    double consigne;
    double value;
    double erreur;
    double command;
    
    double Kp;
    double Ki;
    double Kd;
    double erreurProportionelleMax;
    double erreurIntegraleMax;
    double erreurDeriveeMax;
    
    double erreurIntegrale;
    double epsilon_1;
    
    double corrP;
    double corrI;
    double corrD ;
} PidCorrector;

extern PidCorrector PidX;
extern PidCorrector PidTheta;
void SetupPidAsservissement (volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, double proportionelleMax, double integralMax, double deriveeMax);

//void SetupPidAsservissement(volatile PidCorrector* PidCorr, double Kp, double Ki , double Kd, double proportionelleMax, double integralMax, double deriveeMax);
void AsservissementValeur();

//double Correcteur(volatile PidCorrector* PidCorr, double erreur);
#endif
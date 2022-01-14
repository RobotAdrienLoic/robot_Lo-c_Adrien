#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "asservissement.h"
#include "UART_Protocol.h"
#include "Toolbox.h"
#include "robot.h"
#include "QEI.h"
#include "timer.h"
#include "Utilities.h"


unsigned char asservissementPayload [104];

double consigneX =0;
double consigneTheta = 0;                    
double valueX =0;
double valueTheta =0;
double errorX = 0;
double errorTheta = 0;
double commandX = 0;
double commandTheta = 0;
    //-------------------
double corrPX = 0;
double corrPTheta = 0;
double corrIX = 0;
double corrITheta = 0;
double corrDX = 0;
double corrDTheta = 0;
   //-------------------
double KpX = 0;
double KpTheta = 0; 
double KiX = 0; 
double KiTheta = 0; 
double KdX = 0; 
double KdTheta = 0;
    //-------------------
double corrLimitPX = 0;
double corrLimitPTheta = 0;
double corrLimitIX = 0;
double corrLimitITheta = 0;
double corrLimitDX = 0;
double corrLimitDTheta = 0;

void SetupPidAsservissement (volatile PidCorrector* PidCorr, double Kp, double Ki, double Kd, double proportionelleMax, double integralMax, double deriveeMax)
{
PidCorr -> Kp = Kp;
PidCorr -> erreurProportionelleMax = proportionelleMax ; // on limie la correction due au Kp
PidCorr -> Ki = Ki ;
PidCorr -> erreurIntegraleMax = integralMax ; // on limite la correction due au Ki
PidCorr -> Kd = Kd;
PidCorr -> erreurDeriveeMax = deriveeMax ;
}

void AsservissementValeur(){
    //-------------------
    int nb_octet = 0;
    getBytesFromFloat(asservissementPayload,nb_octet, (float)(robotState.PidX.consigne)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.consigne));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.vitesseLineaireFromOdometry)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.vitesseAngulaireFromOdometry)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.erreur));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.erreur)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.command));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.command)); 
    //-------------------   
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.corrP));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.corrP));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.corrI));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.corrI));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.corrD));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.corrD));
    //-------------------
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.Kp)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.Kp));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.Ki));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.Ki));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.Kd)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.Kd));
    //-------------------
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.erreurProportionelleMax)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.erreurProportionelleMax)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.erreurIntegraleMax)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.erreurIntegraleMax));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidX.erreurDeriveeMax)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.PidTheta.erreurDeriveeMax));

    UartEncodeAndSendMessage(0x0075, nb_octet +=4, asservissementPayload);
}

double Correcteur(volatile PidCorrector* PidCorr,double erreur){
    
    PidCorr ->erreur = erreur;
    double erreurProportionnelle = LimitToInterval(erreur, -PidCorr->erreurProportionelleMax/PidCorr->Kp, PidCorr->erreurProportionelleMax/PidCorr->Kp);
    PidCorr -> corrP = PidCorr-> Kp *erreurProportionnelle;
    
    PidCorr->erreurIntegrale += erreur / FREQ_ECH_QEI;
    PidCorr->erreurIntegrale = LimitToInterval(PidCorr->erreurIntegrale, -PidCorr->erreurIntegraleMax/PidCorr->Ki, PidCorr->erreurIntegraleMax/PidCorr->Ki);
    PidCorr->corrI = PidCorr->erreurIntegrale * PidCorr->Ki;
  
    double erreurDerivee = (erreur - PidCorr->epsilon_1) * FREQ_ECH_QEI;
    double deriveeBornee = LimitToInterval(erreurDerivee, -PidCorr->erreurDeriveeMax/PidCorr->Kd, PidCorr->erreurDeriveeMax/PidCorr->Kd);
    PidCorr->epsilon_1 = erreur;
    PidCorr->corrD = PidCorr->Kd * deriveeBornee;
    
    return PidCorr->corrP + PidCorr->corrI + PidCorr->corrD;
}

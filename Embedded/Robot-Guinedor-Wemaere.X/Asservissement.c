#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include "main.h"
#include "asservissement.h"
#include "UART_Protocol.h"
#include "Toolbox.h"
#include "Robot.h"
#include "QEI.h"
#include "timer.h"
#include "Utilities.h"

PidCorrector PidX;
PidCorrector PidTheta;

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
    getBytesFromFloat(asservissementPayload,nb_octet, (float)(consigneX)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(consigneTheta));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.vitesseLineaireFromOdometry)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(robotState.vitesseAngulaireFromOdometry)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(errorX));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(errorTheta)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(commandX));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(commandTheta)); 
    //-------------------   
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrPX));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrPTheta));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrIX));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrITheta));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrDX));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrDTheta));
    //-------------------
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(PidX.Kp)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(PidTheta.Kp));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(PidX.Ki));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(PidTheta.Ki));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(PidX.Kd)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(PidTheta.Kd));
    //-------------------
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(PidX.erreurProportionelleMax)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrLimitPTheta)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrLimitIX)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrLimitITheta));
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrLimitDX)); 
    getBytesFromFloat(asservissementPayload, nb_octet += 4, (float)(corrLimitDTheta));

    UartEncodeAndSendMessage(0x0075, nb_octet +=4, asservissementPayload);
}
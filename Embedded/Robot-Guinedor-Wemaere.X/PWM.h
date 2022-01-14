#ifndef PWM_H
#define PWM_H
#define MOTEUR_DROIT 0
#define MOTEUR_GAUCHE 1

#define COEFF_VITESSE_ANGULAIRE_PERCENT 75
#define COEFF_VITESSE_LINEAIRE_PERCENT 75

void InitPWM(void);
void PWMUpdateSpeed(void);
//void PWMSetSpeed(float vitesseEnPourcents, int moteur);
void PWMSetSpeedConsigne(float vitesseEnpourcents, int moteur);

#endif /* PWM_H */
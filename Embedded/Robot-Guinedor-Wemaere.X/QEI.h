#ifndef QEI_H
#define QEI_H

#define DISTROUES 0.217
#define FREQ_ECH_QEI 250
#define POSITION_DATA 0x0061
#define point_meter 0.000016336  

void InitQEI1(void);
void InitQEI2(void);
void QEIUpdateData(void);
void SendPositionData(void);

#endif /* QEI_H */

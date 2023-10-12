#ifndef BATT_H
#define BATT_H


void batt_monitor(void);
extern uint16_t adc_batt_raw;
extern uint32_t adc_batt_v;

#endif
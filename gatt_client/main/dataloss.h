#ifndef DATALOSS_H
#define DATALOSS_H

/* Includes */
/* ESP APIs */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void dataloss_new_sample(uint8_t* data, uint16_t data_len, bool* loss, bool* wrong_data_len, bool* counter_out_of_bound);
void dataloss_reset();
bool dataloss_calculate_loss_amplitude(uint32_t last_counter, uint32_t counter);
int dataloss_get_last_loss_amplitude();
float dataloss_get_loss_percentage();
uint32_t dataloss_number_losses();

#endif // DATALOSS_H


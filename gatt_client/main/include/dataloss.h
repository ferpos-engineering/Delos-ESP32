#ifndef DATALOSS_H
#define DATALOSS_H

/* Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* ESP APIs */

void dataloss_init();
void dataloss_new_sample(int slot, uint8_t* data, uint16_t data_len, bool* loss, bool* wrong_data_len, bool* counter_out_of_bound);
void dataloss_reset(int slot);
bool dataloss_calculate_loss_amplitude(int slot, uint32_t last_counter, uint32_t counter);
int dataloss_get_last_loss_amplitude(int slot);
float dataloss_get_loss_percentage(int slot);
uint32_t dataloss_number_losses(int slot);

#endif // DATALOSS_H


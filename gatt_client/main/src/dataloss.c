#include "include/dataloss.h"

#define NUM_SAMPLES 12000
#define NUM_DATA_PER_SAMPLE 16
#define NUM_SLOTS 3
#define countof(x) x * sizeof(uint8_t)

typedef struct {
    uint32_t last_counter;
    uint32_t start_number;
    uint32_t last_number;
    int num_data_losses;
    bool init;
    int amplitude_losses[NUM_SAMPLES];
} dataloss_info_t;

static dataloss_info_t dataloss_info_peers[NUM_SLOTS];

int test()
{
    // uint8_t data[NUM_DATA_PER_SAMPLE] = { 1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0x0a, 0x0b, 0x0c};
    
    // bool loss, wrong_data_len, counter_out_of_bound;
    // dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);
    
    // if(loss)
    // {
    //     int num_samples_lost = dataloss_get_last_loss_amplitude();
    //     printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    // }
    
    // data[0] = 2;
    
    // dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);
    
    // if(loss)
    // {
    //     int num_samples_lost = dataloss_get_last_loss_amplitude();
    //     printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    // }    
    
    // dataloss_reset();
    
    // data[0] = 9;
    
    // dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);
    
    // if(loss)
    // {
    //     int num_samples_lost = dataloss_get_last_loss_amplitude();
    //     printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    // }    
    
    // data[0] = 0xb;
    
    // dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);
    
    // if(loss)
    // {
    //     int num_samples_lost = dataloss_get_last_loss_amplitude();
    //     printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    // }    
    
    // data[0] = 0x0f;
    // data[1] = 0;
    
    // dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);    
    
    // if(loss)
    // {
    //     int num_samples_lost = dataloss_get_last_loss_amplitude();
    //     printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    // }    
    
    
    // printf("Loss percentage %f\r\n", dataloss_get_loss_percentage());
    // printf("Number of losses %lu\r\n", dataloss_number_losses());

    return 0;
}

void dataloss_init()
{
    for(int i = 0; i < NUM_SLOTS; i++)
    {
        dataloss_info_t* dataloss_info = &dataloss_info_peers[i];
        dataloss_info->last_counter = 0;
        dataloss_info->start_number = 1;
        dataloss_info->last_number = NUM_SAMPLES;
        dataloss_info->num_data_losses = 0;
        dataloss_info->init = false;
        memset(&dataloss_info->amplitude_losses[0], 0, sizeof(int) * NUM_SAMPLES);
    }
}

void dataloss_new_sample(int slot, uint8_t* data, uint16_t data_len, bool* loss, bool* wrong_data_len, bool* counter_out_of_bound)
{
    dataloss_info_t* dataloss_info = &dataloss_info_peers[slot];
    *loss = false;
    *wrong_data_len = false;
    *counter_out_of_bound = false;
    
    if(data_len != NUM_DATA_PER_SAMPLE)
    {
        *wrong_data_len = true;
        return;
    }
    
    uint32_t counter;
    memcpy(&counter, data, sizeof(counter));    
    
    if(counter < dataloss_info->start_number || counter > dataloss_info->last_number)
    {
        *counter_out_of_bound = true;
        return;
    }
    
    if(!dataloss_info->init)
    {
        dataloss_info->last_counter = counter;
        dataloss_info->init = true;
        return;
    }

    *loss = dataloss_calculate_loss_amplitude(slot, dataloss_info->last_counter, counter);
    
    dataloss_info->last_counter = counter;
}

void dataloss_reset(int slot)
{
    dataloss_info_t* dataloss_info = &dataloss_info_peers[slot];
    dataloss_info->init = false;
}

bool dataloss_calculate_loss_amplitude(int slot, uint32_t last_counter, uint32_t counter)
{
    dataloss_info_t* dataloss_info = &dataloss_info_peers[slot];
    int amplitude = counter - last_counter - 1;
    
    if(amplitude == 0)
    {
        return false;
    }
    
    dataloss_info->amplitude_losses[dataloss_info->num_data_losses++] = amplitude;
    return true;
}

int dataloss_get_last_loss_amplitude(int slot)
{
    dataloss_info_t* dataloss_info = &dataloss_info_peers[slot];
    if(dataloss_info->num_data_losses == 0)
    {
        return 0;
    }
    
    return dataloss_info->amplitude_losses[dataloss_info->num_data_losses - 1];
}

float dataloss_get_loss_percentage(int slot)
{
    dataloss_info_t* dataloss_info = &dataloss_info_peers[slot];
    int lost_samples = 0;
    for(int i = 0; i < dataloss_info->num_data_losses; i++)
    {
        lost_samples += dataloss_info->amplitude_losses[i];
    }
    
    float total_samples = NUM_SAMPLES;
    float percentage = (lost_samples / total_samples) * 100.0;
    return percentage;
}

uint32_t dataloss_number_losses(int slot)
{
    dataloss_info_t* dataloss_info = &dataloss_info_peers[slot];
    return dataloss_info->num_data_losses;
}
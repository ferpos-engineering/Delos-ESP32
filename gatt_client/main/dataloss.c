#include "dataloss.h"

#define NUM_SAMPLES 12000
#define NUM_DATA_PER_SAMPLE 16
#define countof(x) x * sizeof(uint8_t)

static int num_data_losses = 0;
static int amplitude_losses[NUM_SAMPLES] = { 0 };

static uint32_t last_counter = 0;
static uint32_t start_number = 1;
static uint32_t last_number = NUM_SAMPLES;

static bool init = false;

int test()
{
    uint8_t data[NUM_DATA_PER_SAMPLE] = { 1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0x0a, 0x0b, 0x0c};
    
    bool loss, wrong_data_len, counter_out_of_bound;
    dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);
    
    if(loss)
    {
        int num_samples_lost = dataloss_get_last_loss_amplitude();
        printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    }
    
    data[0] = 2;
    
    dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);
    
    if(loss)
    {
        int num_samples_lost = dataloss_get_last_loss_amplitude();
        printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    }    
    
    dataloss_reset();
    
    data[0] = 9;
    
    dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);
    
    if(loss)
    {
        int num_samples_lost = dataloss_get_last_loss_amplitude();
        printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    }    
    
    data[0] = 0xb;
    
    dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);
    
    if(loss)
    {
        int num_samples_lost = dataloss_get_last_loss_amplitude();
        printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    }    
    
    data[0] = 0x0f;
    data[1] = 0;
    
    dataloss_new_sample(data, countof(NUM_DATA_PER_SAMPLE), &loss, &wrong_data_len, &counter_out_of_bound);    
    
    if(loss)
    {
        int num_samples_lost = dataloss_get_last_loss_amplitude();
        printf("DATA LOSS, lost %d samples\r\n", num_samples_lost);
    }    
    
    
    printf("Loss percentage %f\r\n", dataloss_get_loss_percentage());
    printf("Number of losses %lu\r\n", dataloss_number_losses());

    return 0;
}

void dataloss_new_sample(uint8_t* data, uint16_t data_len, bool* loss, bool* wrong_data_len, bool* counter_out_of_bound)
{
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
    
    if(counter < start_number || counter > last_number)
    {
        *counter_out_of_bound = true;
        return;
    }
    
    if(!init)
    {
        last_counter = counter;
        init = true;
        return;
    }

    *loss = dataloss_calculate_loss_amplitude(last_counter, counter);
    
    last_counter = counter;
}

void dataloss_reset()
{
    init = false;
}

bool dataloss_calculate_loss_amplitude(uint32_t last_counter, uint32_t counter)
{
    int amplitude = counter - last_counter - 1;
    
    if(amplitude == 0)
    {
        return false;
    }
    
    amplitude_losses[num_data_losses++] = amplitude;
    return true;
}

int dataloss_get_last_loss_amplitude()
{
    if(num_data_losses == 0)
    {
        return 0;
    }
    
    return amplitude_losses[num_data_losses - 1];
}

float dataloss_get_loss_percentage()
{
    int lost_samples = 0;
    for(int i = 0; i < num_data_losses; i++)
    {
        lost_samples += amplitude_losses[i];
    }
    
    float total_samples = NUM_SAMPLES;
    float percentage = (lost_samples / total_samples) * 100.0;
    return percentage;
}

uint32_t dataloss_number_losses()
{
    return num_data_losses;
}
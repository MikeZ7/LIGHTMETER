#ifndef INC_ZENIT_ET_H_
#define INC_ZENIT_ET_H_

#define APERTURE_F_TABLE_SIZE 7
#define APERTURE_TIME_TABLE_SIZE 5
uint16_t exposure_times_list[APERTURE_TIME_TABLE_SIZE] = {30, 60, 125, 250, 500};
double apertures_f_list[APERTURE_F_TABLE_SIZE] = {2.0, 2.8, 4.0, 5.6, 8.0, 11.0, 16.0};

#endif /* INC_ZENIT_ET_H_ */

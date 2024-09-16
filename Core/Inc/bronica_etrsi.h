#ifndef INC_BRONICA_ETRSI_H_
#define INC_BRONICA_ETRSI_H_

#define APERTURE_F_TABLE_SIZE 7
#define APERTURE_TIME_TABLE_SIZE 7
uint16_t exposure_times_list[APERTURE_TIME_TABLE_SIZE] =  {8, 15, 30, 60, 125, 250, 500};
double apertures_f_list[APERTURE_F_TABLE_SIZE] = {2.8, 4.0, 5.6, 8.0, 11.0, 16.0, 22.0};

#endif /* INC_BRONICA_ETRSI_H_ */

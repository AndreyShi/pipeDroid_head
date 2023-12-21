/*
 * track.h
 *
 *  Created on: 23 èþë. 2020 ã.
 *      Author: A-User
 */

#ifndef SOURCE_APPLICATION_TRACK_H_
#define SOURCE_APPLICATION_TRACK_H_
#include <stdint.h>
void track_scan_fwd(void);
void track_scan_rev(void);
void track_stop(void);
void track_fwd(void);
void track_rev(void);
void track_update_speed(uint16_t speed, float* kSpeed);
void track_update_scan_speed(uint16_t speed,float* kSpeed);
void track_update_step(uint16_t step, float* kSpeed);
void track_update_acs(uint16_t acs, float* kAcs);
void track_update_Ilimit(uint16_t* Ilimit);

#endif /* SOURCE_APPLICATION_TRACK_H_ */

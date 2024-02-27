/*
this file for mux control
*/
#ifndef MUX_H
#define MUX_H

void spi_mux_config(void);
void mux_set_sync(char on);
void mux_reset(char on);
void spi_mux_send(unsigned char byte);
char wait_spi(void);
void set_muxes(const char* data);
#endif
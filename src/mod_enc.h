#ifndef MOD_ENC_H
#define MOD_ENC_H

#include "stdlib.h"

void mod_enc_configure_hardware(void);

int32_t mod_enc_get_countA(void);
int32_t mod_enc_get_countB(void);

void mod_enc_reset_countA(void);
void mod_enc_reset_countB(void);

#endif
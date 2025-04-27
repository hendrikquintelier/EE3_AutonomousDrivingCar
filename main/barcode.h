#ifndef BARCODE_H
#define BARCODE_H

#include <stdbool.h>

/** \file barcode.h – runtime IR‑barcode helper
 *
 * The helpers are completely self‑contained.  Call the three public
 * functions from your main drive loop without blocking or extra RAM
 * allocations.
 *
 * Usage pattern:
 * --------------------------------------------------------------
 *  barcode_reset();                         // before you start driving
 *  while(driving) {
 *      float d_cm = encoder_get_distance();
 *      float v    = read_ir_voltage();
 *      barcode_update(d_cm, v);             // ~50 Hz
 *  }
 *  if (barcode_is_complete()) {
 *      log_remote("TAG = %s", barcode_get_bits());
 *  }
 */

/* Reset internal state so the next call to barcode_update() starts a
 * fresh grey‑band measurement.                                          */
void barcode_reset(void);

/* Feed one sample (distance in cm, IR voltage in V).
 * Call from your control loop every cycle.                              */
void barcode_update(float distance_cm, float ir_voltage);

/* Returns true once the barcode has been fully recorded and decoded
 * (you can then fetch it with barcode_get_bits()).                      */
bool barcode_is_complete(void);

/* Returns pointer to the 8‑char ‘0’/‘1’ string (NUL‑terminated).
 * Only valid if barcode_is_complete() == true.                           */
const char *barcode_get_bits(void);

void bc_finish(void);

#endif /* BARCODE_H */
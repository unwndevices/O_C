/*
 *
 * compile options.
 *
 */

#ifndef OC_OPTIONS_H_
#define OC_OPTIONS_H_

/* ------------ uncomment for Buchla/non-octaval/quartertone support: -------------------------------  */
//#define BUCHLA_SUPPORT
/* ------------ uncomment for use with Northernlight cOC program card: ------------------------------  */
//#define BUCHLA_cOC
/* ------------ uncomment for use with Northernlight 4U / cOC : -------------------------------------  */
//#define BUCHLA_4U
/* ------------ uncomment for boring app names ------------------------------------------------------  */
//#define BORING_APP_NAMES
/* ------------ print debug messages to USB serial --------------------------------------------------  */
//#define PRINT_DEBUG
/* ------------ flip screen / IO mapping ------------------------------------------------------------  */
#define FLIP_180_SCREEN
/* ------------ invert screen pixels ----------------------------------------------------------------  */
//#define INVERT_DISPLAY
/* ------------ use DAC8564 -------------------------------------------------------------------------  */
//#define DAC8564
/* ------------ 0 / 10V range -----------------------------------------------------------------------  */
//#define IO_10V

#if defined(IO_10V)
  #define BUCHLA_4U
#endif

#endif

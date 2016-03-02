/* Nicolas Kniebihli INP-T ENSEEIHT
*/
#ifndef LOG_H_
#define LOG_H_

#include <stdlib.h>
#include <stdio.h>

FILE* fichier;

void log_init();
void log_close();
void log_add(const char * string);

#endif

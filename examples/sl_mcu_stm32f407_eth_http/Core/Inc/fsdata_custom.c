/*
 * fsdata_custom.c
 *
 *  Created on: Jun 24, 2026
 *      Author: lth
 */


#include "lwip/apps/fs.h"
#include <string.h>

// Adding 'static' prevents the duplicate definition linker error
static const struct fsdata_file file__index_html[] = {{
    NULL,
    (const unsigned char *)"/index.html",
    (const unsigned char *)"Dummy",
    5,
    0
}};

#define FS_ROOT file__index_html

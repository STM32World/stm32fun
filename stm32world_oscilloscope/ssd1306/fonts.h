
#include "stdint.h"


#ifndef _fonts_h
#define _fonts_h

//
// Structure om font te definieren
//
typedef struct {
  const uint8_t FontWidth;    /*!< Font width in pixels */
  uint8_t FontHeight;         /*!< Font height in pixels */
  const uint16_t *data;       /*!< Pointer to data font data array */
} FontDef;


//
// De 3 fonts
//
extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;

#endif // _fonts_h

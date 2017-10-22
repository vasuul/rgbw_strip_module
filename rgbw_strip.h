#ifndef __RGBW_STRIP_H
#define __RGBW_STRIP_H

typedef struct rgbw_led {
  uint8_t r,g,b,w;
} rgbw_led_t;

typedef struct rgbw_render {
  uint offset;
  uint count;
  rgbw_led_t leds[];
} rgbw_render_t;

#define RGBW_STRIP_RENDER       _IOW('s', 1, rgbw_render_t *)
#define RGBW_STRIP_GET_NUM_LEDS _IOR('s', 2, int *)

#endif

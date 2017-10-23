#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

#include "../rgbw_strip.h"

int main(int argc, char ** argv) {
  int fd = open("/dev/rgbws", O_RDWR);
  int num_leds;
  rgbw_render_t *r = malloc(sizeof(rgbw_render_t) + sizeof(rgbw_led_t) * 2);

  printf("fd = %d\n", fd);  

  r->offset = 1;
  r->count = 2;

  r->leds[0].r = 200;
  r->leds[0].w = 125;
  r->leds[1].g = 250;

  ioctl(fd, RGBW_STRIP_GET_NUM_LEDS, &num_leds);
  printf("Number of leds: %d\n", num_leds);
  
  ioctl(fd, RGBW_STRIP_RENDER, r);
}

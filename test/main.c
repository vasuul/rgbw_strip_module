#include <sys/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>

#include "../rgbw_strip.h"

int main(int argc, char ** argv) {

  if(argc < 7) {
    printf("Need more arguments: %s <r> <g> <b> <w> <start> <end>\n", argv[0]);
    exit(1);
  }

  int i;
  int rv = atoi(argv[1]);
  int gv = atoi(argv[2]);
  int bv = atoi(argv[3]);
  int wv = atoi(argv[4]);

  int start = atoi(argv[5]);
  int count = atoi(argv[6]);

  int fd = open("/dev/rgbws", O_RDWR);
  int num_leds;
  
  rgbw_render_t *r = malloc(sizeof(rgbw_render_t) + sizeof(rgbw_led_t) * count);

  printf("fd = %d\n", fd);  

  r->offset = start;
  r->count = count;

  for(i = 0; i < count; i++) {
    r->leds[i].r = rv;
    r->leds[i].g = gv;
    r->leds[i].b = bv;
    r->leds[i].w = wv;
  }

  ioctl(fd, RGBW_STRIP_GET_NUM_LEDS, &num_leds);
  printf("Number of leds: %d\n", num_leds);
  
  ioctl(fd, RGBW_STRIP_RENDER, r);
}

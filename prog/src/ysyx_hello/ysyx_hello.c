#include "ysyx.h"
#include <am.h>
#include <klib-macros.h>
#include <klib.h>

int main() {
  timer_init(1, 100000);
  printf("\n");
  printf("Hello test\n");
  printf("after delay will clear screen\n");
  delay_ms(1000);
  printf("\033[2J\033[H");
  printf("Hello test\n");
  return 0;
}

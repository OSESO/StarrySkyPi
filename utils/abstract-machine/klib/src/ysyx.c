#include "am.h"
#include "klib-macros.h"
#include "klib.h"
#include "stdint.h"
#include <stdint.h>
#include <ysyx.h>

// void core_init() {
//   // GPIO_0
//   mmio_write(GPIO_0_BASE_ADDR + GPIO_0_REG_PADDIR_OFFSET, 0x00000000);
//   mmio_write(GPIO_0_BASE_ADDR + GPIO_0_REG_PADOUT_OFFSET, 0x00000000);
//   mmio_write(GPIO_0_BASE_ADDR + GPIO_0_REG_INTEN_OFFSET, 0x00000000);
//   mmio_write(GPIO_0_BASE_ADDR + GPIO_0_REG_INTTYPE0_OFFSET, 0x00000000);
//   mmio_write(GPIO_0_BASE_ADDR + GPIO_0_REG_INTTYPE1_OFFSET, 0x00000000);
//   mmio_write(GPIO_0_BASE_ADDR + GPIO_0_REG_INTSTAT_OFFSET, 0x00000000);
//   mmio_write(GPIO_0_BASE_ADDR + GPIO_0_REG_IOFCFG_OFFSET, 0x00000000);
//   mmio_write(GPIO_0_BASE_ADDR + GPIO_0_REG_PINMUX_OFFSET, 0x00000000);

//   // GPIO_1
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_PADDIR_OFFSET, 0x00000000);
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_PADOUT_OFFSET, 0x00000000);
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_INTEN_OFFSET, 0x00000000);
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_INTTYPE0_OFFSET, 0x00000000);
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_INTTYPE1_OFFSET, 0x00000000);
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_INTSTAT_OFFSET, 0x00000000);
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET, 0x00000000);
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_PINMUX_OFFSET, 0x00000000);

//   // PWM_0
//   mmio_write(PWM_0_BASE_ADDR + PWM_0_REG_CTRL_OFFSET, 0x00000000);
//   mmio_write(PWM_0_BASE_ADDR + PWM_0_REG_PSCR_OFFSET, 0x00000002);
//   mmio_write(PWM_0_BASE_ADDR + PWM_0_REG_CMP_OFFSET, 0x00000000);
//   mmio_write(PWM_0_BASE_ADDR + PWM_0_REG_CR0_OFFSET, 0x00000000);
//   mmio_write(PWM_0_BASE_ADDR + PWM_0_REG_CR1_OFFSET, 0x00000000);
//   mmio_write(PWM_0_BASE_ADDR + PWM_0_REG_CR2_OFFSET, 0x00000000);
//   mmio_write(PWM_0_BASE_ADDR + PWM_0_REG_CR3_OFFSET, 0x00000000);
//   mmio_write(PWM_0_BASE_ADDR + PWM_0_REG_STAT_OFFSET, 0x00000000);

//   // PWM_1
//   mmio_write(PWM_1_BASE_ADDR + PWM_1_REG_CTRL_OFFSET, 0x00000000);
//   mmio_write(PWM_1_BASE_ADDR + PWM_1_REG_PSCR_OFFSET, 0x00000002);
//   mmio_write(PWM_1_BASE_ADDR + PWM_1_REG_CMP_OFFSET, 0x00000000);
//   mmio_write(PWM_1_BASE_ADDR + PWM_1_REG_CR0_OFFSET, 0x00000000);
//   mmio_write(PWM_1_BASE_ADDR + PWM_1_REG_CR1_OFFSET, 0x00000000);
//   mmio_write(PWM_1_BASE_ADDR + PWM_1_REG_CR2_OFFSET, 0x00000000);
//   mmio_write(PWM_1_BASE_ADDR + PWM_1_REG_CR3_OFFSET, 0x00000000);
//   mmio_write(PWM_1_BASE_ADDR + PWM_1_REG_STAT_OFFSET, 0x00000000);

//   // PWM_2
//   mmio_write(PWM_2_BASE_ADDR + PWM_2_REG_CTRL_OFFSET, 0x00000000);
//   mmio_write(PWM_2_BASE_ADDR + PWM_2_REG_PSCR_OFFSET, 0x00000002);
//   mmio_write(PWM_2_BASE_ADDR + PWM_2_REG_CMP_OFFSET, 0x00000000);
//   mmio_write(PWM_2_BASE_ADDR + PWM_2_REG_CR0_OFFSET, 0x00000000);
//   mmio_write(PWM_2_BASE_ADDR + PWM_2_REG_CR1_OFFSET, 0x00000000);
//   mmio_write(PWM_2_BASE_ADDR + PWM_2_REG_CR2_OFFSET, 0x00000000);
//   mmio_write(PWM_2_BASE_ADDR + PWM_2_REG_CR3_OFFSET, 0x00000000);
//   mmio_write(PWM_2_BASE_ADDR + PWM_2_REG_STAT_OFFSET, 0x00000000);

//   // RCU
//   // mmio_write(RCU_BASE_ADDR + RCU_REG_CTRL_OFFSET, 0x00000000);
//   // mmio_write(RCU_BASE_ADDR + RCU_REG_RDIV_OFFSET, 0x00000000);
//   // mmio_write(RCU_BASE_ADDR + RCU_REG_STAT_OFFSET, 0x00000000);

//   // WDG
//   mmio_write(WDG_BASE_ADDR + WDG_REG_CTRL_OFFSET, 0x00000000);
//   mmio_write(WDG_BASE_ADDR + WDG_REG_PSCR_OFFSET, 0x00000002);
//   // mmio_write(WDG_BASE_ADDR + WDG_REG_CNT_OFFSET, 0x00000000);
//   mmio_write(WDG_BASE_ADDR + WDG_REG_CMP_OFFSET, 0x00000000);
//   mmio_write(WDG_BASE_ADDR + WDG_REG_STAT_OFFSET, 0x00000000);
//   mmio_write(WDG_BASE_ADDR + WDG_REG_KEY_OFFSET, 0x00000000);
//   mmio_write(WDG_BASE_ADDR + WDG_REG_FEED_OFFSET, 0x00000000);

//   // I2C
//   I2C_REG_CTRL = 0x00;
//   I2C_REG_PSCRL = 0xFF;
//   I2C_REG_PSCRH = 0x00;
//   I2C_REG_TXR = 0x00;
//   I2C_REG_RXR = 0x00;
//   I2C_REG_CMD = 0x00;
//   I2C_REG_SR = 0x00;

//   // RTC
//   mmio_write(RTC_BASE_ADDR + RTC_REG_CTRL_OFFSET, 0x00000000);
//   mmio_write(RTC_BASE_ADDR + RTC_REG_PSCR_OFFSET, 0x00000002);
//   mmio_write(RTC_BASE_ADDR + RTC_REG_ALRM_OFFSET, 0x00000000);
//   mmio_write(RTC_BASE_ADDR + RTC_REG_ISTA_OFFSET, 0x00000000);
//   mmio_write(RTC_BASE_ADDR + RTC_REG_SSTA_OFFSET, 0x00000000);

//   // CLINT
//   mmio_write(CLINT_BASE_ADDR + CLINT_REG_MSIP_OFFSET, 0x00000000);
//   mmio_write(CLINT_BASE_ADDR + CLINT_REG_MTIMEL_OFFSET, 0x00000000);
//   mmio_write(CLINT_BASE_ADDR + CLINT_REG_MTIMEH_OFFSET, 0x00000000);
//   mmio_write(CLINT_BASE_ADDR + CLINT_REG_MTIMECMPL_OFFSET, 0xFFFFFFFF);
//   mmio_write(CLINT_BASE_ADDR + CLINT_REG_MTIMECMPH_OFFSET, 0xFFFFFFFF);

//   // PS2
//   mmio_write(PS2_BASE_ADDR + PS2_REG_CTRL_OFFSET, 0x00000000);
//   mmio_write(PS2_BASE_ADDR + PS2_REG_DATA_OFFSET, 0x00000000);
//   mmio_write(PS2_BASE_ADDR + PS2_REG_STAT_OFFSET, 0x00000000);

//   // // SPI1
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_STATUS_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_CLKDIV_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_CMD_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_ADR_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_LEN_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_DUM_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_TXFIFO_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_RXFIFO_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_INTCFG_OFFSET, 0x00000000);
//   // mmio_write(SPI1_BASE_ADDR + SPI1_REG_INTSTA_OFFSET, 0x00000000);
// }

void info_id() {
  uint32_t mvendorid;
  __asm__ volatile("csrr %0, mvendorid" : "=r"(mvendorid));
  uint64_t marchid;
  __asm__ volatile("csrr %0, marchid" : "=r"(marchid));
  uint64_t mimpid;
  __asm__ volatile("csrr %0, mimpid" : "=r"(mimpid));

  char mvendorid_ch[5];
  for (int i = 0; i < 4; i++) {
    mvendorid_ch[i] = (mvendorid >> (i * 8)) & 0xFF;
  }
  mvendorid_ch[4] = '\0';
  char marchid_ch[8];
  for (int i = 0; i < 8; i++) {
    marchid_ch[i] = (marchid >> (i * 8)) & 0xFF;
  }
  char mimpid_ch[8];
  for (int i = 0; i < 8; i++) {
    mimpid_ch[i] = (mimpid >> (i * 8)) & 0xFF;
  }

  printf("\033[34m\n************ ID_INFO ************\n\033[0m");
  printf("\033[34mmvendorid_ch: \t%8s\n\033[0m", mvendorid_ch);
  printf("\033[34mmarchid_ch: \t%8s\n\033[0m", marchid_ch);
  printf("\033[34mmimpid_ch: \t%8s\n\033[0m", mimpid_ch);
  printf("\033[34m*********************************\n\n\033[0m");
}

void timer_init(uint32_t div, uint32_t cmp) {
  printf("TIMER INIT:\n");
  TIMER_0_REG_CTRL = (uint32_t)0x0; // disable timer
  while (TIMER_0_REG_STAT == 1)
    ; // clear irq
  TIMER_0_REG_PSCR = div - 1;
  TIMER_0_REG_CMP = cmp - 1;

  printf("CTRL: %d PSCR: %d CMP: %d\n", TIMER_0_REG_CTRL, TIMER_0_REG_PSCR,
         TIMER_0_REG_CMP);
  printf("TIMER INIT DONE\n");
}

void delay_ms(uint32_t val) {
  TIMER_0_REG_CTRL = (uint32_t)0xD;
  for (int i = 0; i < val; ++i) {
    while (TIMER_0_REG_STAT == 0)
      ;
  }
  TIMER_0_REG_CTRL = (uint32_t)0xD;
}

// spi1
void spi1_wr_dat(uint8_t dat) {
  SPI1_REG_LEN = 0x80000;
  *((volatile uint8_t *)(SPI1_BASE_ADDR + SPI1_REG_TXFIFO_OFFSET + 3)) = (uint8_t)dat;
  *((volatile uint8_t *)(SPI1_BASE_ADDR + SPI1_REG_STATUS_OFFSET + 1)) = 1;
  *((volatile uint8_t *)(SPI1_BASE_ADDR + SPI1_REG_STATUS_OFFSET + 0)) = 2;
  while ((SPI1_REG_STATUS & 0xFFFF) != 1)
    ;
}

// // spi1_tft
// void spi_tft_init() {
//   printf("GPIO INIT:\n");
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET,
//              mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET) |
//                  (uint32_t)(1 << 0)); // SCK
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET,
//              mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET) |
//                  (uint32_t)(1 << 1)); // CS
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET,
//              mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET) |
//                  (uint32_t)(1 << 2)); // MOSI
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET,
//              mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET) |
//                  (uint32_t)(1 << 3)); //
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET,
//              mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET) |
//                  (uint32_t)(1 << 4)); //
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET,
//              mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET) |
//                  (uint32_t)(1 << 5)); //
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_PINMUX_OFFSET,
//              mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_PINMUX_OFFSET) |
//                  0); // FUNC0
//   mmio_write(GPIO_1_BASE_ADDR + GPIO_1_REG_PADDIR_OFFSET,
//              mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_PADDIR_OFFSET) |
//                  (uint32_t)(1 << 6)); // DC
//   printf("GPIO_1_PADDIR: %08x\n",
//          mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_PADDIR_OFFSET));
//   printf("GPIO_1_IOFCFG: %08x\n",
//          mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_IOFCFG_OFFSET));
//   printf("GPIO_1_PINMUX: %08x\n",
//          mmio_read(GPIO_1_BASE_ADDR + GPIO_1_REG_PINMUX_OFFSET));
//   printf("GPIO INIT DONE\n");
//   printf("SPI INIT:\n");
//   mmio_write(SPI1_BASE_ADDR + SPI1_REG_STATUS_OFFSET, (uint32_t)0b10000);
//   mmio_write(SPI1_BASE_ADDR + SPI1_REG_STATUS_OFFSET, (uint32_t)0b00000);
//   mmio_write(SPI1_BASE_ADDR + SPI1_REG_INTCFG_OFFSET, (uint32_t)0b00000);
//   mmio_write(SPI1_BASE_ADDR + SPI1_REG_DUM_OFFSET, (uint32_t)0);
//   mmio_write(SPI1_BASE_ADDR + SPI1_REG_CLKDIV_OFFSET,
//              (uint32_t)3); // sck = apb_clk/2(div+1) 100MHz/2 = 50MHz
//   printf("SPI1_STATUS: %08x\n",
//          mmio_read(SPI1_BASE_ADDR + SPI1_REG_STATUS_OFFSET));
//   printf("SPI1_CLKDIV: %08x\n",
//          mmio_read(SPI1_BASE_ADDR + SPI1_REG_CLKDIV_OFFSET));
//   printf("SPI1_INTCFG: %08x\n",
//          mmio_read(SPI1_BASE_ADDR + SPI1_REG_INTCFG_OFFSET));
//   printf("SPI1_DUM: %08x\n", mmio_read(SPI1_BASE_ADDR +
//   SPI1_REG_DUM_OFFSET)); printf("SPI INIT DONE\n"); printf("tft init
//   begin\n");

//   delay_ms(500);

//   printf("exit sleep\n");
//   lcd_wr_cmd(0x11); // 睡眠退出
//   delay_ms(500);

//   printf("exit sleep\n");
//   // ST7735R 帧速率
//   lcd_wr_cmd(0xB1);
//   lcd_wr_data8(0x01);
//   lcd_wr_data8(0x2C);
//   lcd_wr_data8(0x2D);
//   lcd_wr_cmd(0xB2);
//   lcd_wr_data8(0x01);
//   lcd_wr_data8(0x2C);
//   lcd_wr_data8(0x2D);
//   lcd_wr_cmd(0xB3);
//   lcd_wr_data8(0x01);
//   lcd_wr_data8(0x2C);
//   lcd_wr_data8(0x2D);
//   lcd_wr_data8(0x01);
//   lcd_wr_data8(0x2C);
//   lcd_wr_data8(0x2D);
//   lcd_wr_cmd(0xB4); // 列反转
//   lcd_wr_data8(0x07);
//   printf("after frame rate init\n");

//   // ST7735R Power Sequence
//   lcd_wr_cmd(0xC0);
//   lcd_wr_data8(0xA2);
//   lcd_wr_data8(0x02);
//   lcd_wr_data8(0x84);
//   lcd_wr_cmd(0xC1);
//   lcd_wr_data8(0xC5);
//   lcd_wr_cmd(0xC2);
//   lcd_wr_data8(0x0A);
//   lcd_wr_data8(0x00);
//   lcd_wr_cmd(0xC3);
//   lcd_wr_data8(0x8A);
//   lcd_wr_data8(0x2A);
//   lcd_wr_cmd(0xC4);
//   lcd_wr_data8(0x8A);
//   lcd_wr_data8(0xEE);
//   lcd_wr_cmd(0xC5); // VCOM
//   lcd_wr_data8(0x0E);
//   lcd_wr_cmd(0x36); // MX,MY,RGB mode
//   printf("after power sequence init\n");

//   switch (
//       USE_HORIZONTAL) //
//       显示的方向(竖屏:0,横屏:1,竖屏旋转180度:2,横屏旋转180度:3)
//   {
//   case 0:
//     lcd_wr_data8(0xC8);
//     break; // 竖屏
//   case 1:
//     lcd_wr_data8(0xA8);
//     break; // 横屏
//   case 2:
//     lcd_wr_data8(0x08);
//     break; // 竖屏翻转180度
//   default:
//     lcd_wr_data8(0x68);
//     break; // 横屏翻转180度
//   }

//   // ST7735R Gamma Sequence
//   lcd_wr_cmd(0xE0);
//   lcd_wr_data8(0x0F);
//   lcd_wr_data8(0x1A);
//   lcd_wr_data8(0x0F);
//   lcd_wr_data8(0x18);
//   lcd_wr_data8(0x2F);
//   lcd_wr_data8(0x28);
//   lcd_wr_data8(0x20);
//   lcd_wr_data8(0x22);
//   lcd_wr_data8(0x1F);
//   lcd_wr_data8(0x1B);
//   lcd_wr_data8(0x23);
//   lcd_wr_data8(0x37);
//   lcd_wr_data8(0x00);
//   lcd_wr_data8(0x07);
//   lcd_wr_data8(0x02);
//   lcd_wr_data8(0x10);

//   lcd_wr_cmd(0xE1);
//   lcd_wr_data8(0x0F);
//   lcd_wr_data8(0x1B);
//   lcd_wr_data8(0x0F);
//   lcd_wr_data8(0x17);
//   lcd_wr_data8(0x33);
//   lcd_wr_data8(0x2C);
//   lcd_wr_data8(0x29);
//   lcd_wr_data8(0x2E);
//   lcd_wr_data8(0x30);
//   lcd_wr_data8(0x30);
//   lcd_wr_data8(0x39);
//   lcd_wr_data8(0x3F);
//   lcd_wr_data8(0x00);
//   lcd_wr_data8(0x07);
//   lcd_wr_data8(0x03);
//   lcd_wr_data8(0x10);
//   printf("after gamma sequence init\n");

//   lcd_wr_cmd(0xF0); // 启动测试命令
//   lcd_wr_data8(0x01);
//   lcd_wr_cmd(0xF6); // 禁用ram省电模式
//   lcd_wr_data8(0x00);

//   lcd_wr_cmd(0x3A); // 65k mode
//   lcd_wr_data8(0x05);
//   lcd_wr_cmd(0x29); // 开启显示
//   lcd_fill(0, 0, 128, 128, 0xFFFF);
//   printf("TFT init done\n");
// }

// void lcd_fill_bmp(const uint8_t *bmp, uint16_t x, uint16_t y, uint16_t w,
//                   uint16_t h) {
//   lcd_addr_set(x, y, x + w - 1, y + h - 1);
//   for (uint16_t i = 0; i < h * w * 2; i++) {
//     lcd_wr_data8(bmp[i]);
//   }
// }

// void lcd_refresh(uint16_t *gdm, uint16_t x, uint16_t y, uint16_t w,
//                  uint16_t h) {
//   lcd_addr_set(x, y, x + w - 1, y + h - 1);
//   for (uint16_t i = 0; i < h; i++) {
//     for (uint16_t j = 0; j < w; j++) {
//       lcd_wr_data8(gdm[i * 128 + j] >> 8);
//       lcd_wr_data8(gdm[i * 128 + j]);
//     }
//   }
// }

// void lcd_fill(uint16_t xsta, uint16_t ysta, uint16_t xend, uint16_t yend,
//               uint32_t color) {
//   lcd_addr_set(xsta, ysta, xend - 1, yend - 1);
//   for (uint16_t i = ysta; i < yend; ++i) {
//     for (uint16_t j = xsta; j < xend; ++j) {
//       lcd_wr_data8(color >> 8);
//       lcd_wr_data8(color);
//     }
//   }
// }

// #define LCD_X_OFFSET 2
// #define LCD_Y_OFFSET 3
// void lcd_addr_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
//   lcd_wr_cmd(0x2A);
//   lcd_wr_data8(0x00);
//   lcd_wr_data8(x1 + LCD_X_OFFSET);
//   lcd_wr_data8(0x00);
//   lcd_wr_data8(x2 + LCD_X_OFFSET);
//   lcd_wr_cmd(0x2B);
//   lcd_wr_data8(0x00);
//   lcd_wr_data8(y1 + LCD_Y_OFFSET);
//   lcd_wr_data8(0x00);
//   lcd_wr_data8(y2 + LCD_Y_OFFSET);
//   lcd_wr_cmd(0x2C);
// }

// void lcd_wr_cmd(uint8_t cmd) {
//   lcd_dc_clr;
//   spi1_wr_dat(cmd);
// }

// void lcd_wr_data8(uint8_t dat) {
//   lcd_dc_set;
//   spi1_wr_dat(dat);
// }

// i2c
void i2c_config() {
  GPIO_0_REG_IOFCFG = GPIO_0_REG_IOFCFG | (1 << 29);
  GPIO_0_REG_IOFCFG = GPIO_0_REG_IOFCFG | (1 << 30);
  GPIO_0_REG_PINMUX = GPIO_0_REG_PINMUX_OFFSET | (1 << 29) | (1 << 30);
  printf("GPIO_0_PADDIR: %08x\n", GPIO_0_REG_IOFCFG);
  printf("GPIO_0_PINMUX: %08x\n", GPIO_0_REG_PINMUX);
  I2C_REG_CTRL = (uint8_t)0;
  I2C_REG_PSCRH = (uint8_t)0;
  I2C_REG_PSCRL = (uint8_t)(100 - 1); // 100MHz / (5 * 100KHz) - 1

  printf("CTRL: %08x PSCR: %d\n", I2C_REG_CTRL, I2C_REG_PSCRL);
  I2C_REG_CTRL = (uint8_t)0x80; // core en
  printf("CTRL: %08x PSCR: %d\n", I2C_REG_CTRL, I2C_REG_PSCRL);
  printf("status: %08x\n", I2C_REG_SR);
}

uint32_t i2c_get_ack() {
  while ((I2C_REG_SR & I2C_STATUS_TIP) == 0)
    ;
  while ((I2C_REG_SR & I2C_STATUS_TIP) != 0)
    ;
  return !(I2C_REG_SR & I2C_STATUS_RXACK); // invert since signal is active low
}

uint32_t i2c_busy() {
  return ((I2C_REG_SR & I2C_STATUS_BUSY) == I2C_STATUS_BUSY);
}

void i2c_wr_start(uint32_t slv_addr) {
  I2C_REG_TXR = slv_addr;
  I2C_REG_CMD = I2C_TEST_START_WRITE;
  if (!i2c_get_ack())
    putstr("[wr start]no ack recv\n");
}

void i2c_rd_start(uint32_t slv_addr) {
  do {
    I2C_REG_TXR = slv_addr;
    I2C_REG_CMD = I2C_TEST_START_WRITE;
  } while (!i2c_get_ack());
}

void i2c_write(uint8_t val) {
  I2C_REG_TXR = val;
  I2C_REG_CMD = I2C_TEST_WRITE;
  if (!i2c_get_ack())
    putstr("[i2c write]no ack recv\n");
}

uint32_t i2c_read(uint32_t cmd) {
  I2C_REG_CMD = cmd;
  if (!i2c_get_ack())
    putstr("[i2c read]no ack recv\n");
  return I2C_REG_RXR;
}

void i2c_stop() {
  I2C_REG_CMD = I2C_TEST_STOP;
  while (i2c_busy())
    ;
}

void i2c_wr_nbyte(uint8_t slv_addr, uint16_t reg_addr, uint8_t type,
                  uint8_t num, uint8_t *data) {
  i2c_rd_start(slv_addr);
  if (type == I2C_DEV_ADDR_16BIT) {
    i2c_write((uint8_t)((reg_addr >> 8) & 0xFF));
    i2c_write((uint8_t)(reg_addr & 0xFF));
  } else {
    i2c_write((uint8_t)(reg_addr & 0xFF));
  }
  for (int i = 0; i < num; ++i) {
    i2c_write(*data);
    ++data;
  }
  i2c_stop();
}

void i2c_rd_nbyte(uint8_t slv_addr, uint16_t reg_addr, uint8_t type,
                  uint8_t num, uint8_t *data) {
  i2c_rd_start(slv_addr);
  if (type == I2C_DEV_ADDR_16BIT) {
    i2c_write((uint8_t)((reg_addr >> 8) & 0xFF));
    i2c_write((uint8_t)(reg_addr & 0xFF));
  } else {
    i2c_write((uint8_t)(reg_addr & 0xFF));
  }
  i2c_stop();

  i2c_wr_start(slv_addr + 1);
  for (int i = 0; i < num; ++i) {
    if (i == num - 1)
      data[i] = i2c_read(I2C_TEST_STOP_READ);
    else
      data[i] = i2c_read(I2C_TEST_READ);
  }
}

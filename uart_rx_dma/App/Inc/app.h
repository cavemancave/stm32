/**
 * @file    app.h
 * @brief   应用入口：把 ring / parser / protocol / UART 串起来。
 */
#ifndef APP_H
#define APP_H

void app_init(void);
void app_poll(void);

#endif /* APP_H */

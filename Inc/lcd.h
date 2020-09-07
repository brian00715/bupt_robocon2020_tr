#ifndef __lcd_H
#define __lcd_H
#ifdef __cplusplus
 extern "C" {
#endif
typedef struct{
    char tag[4];
    void* value;
}Lcd_Msg;
int lcd_add(char tag[],void* value_pointer);
void lcd_init();
void lcd_exe();
void lcd_send_flag();
void lcd_send_var();



#ifdef __cplusplus
}
#endif
#endif /*__ lcd_H */
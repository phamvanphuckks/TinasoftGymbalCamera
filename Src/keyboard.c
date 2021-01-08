#include "keyboard.h"
unsigned char Led_code[]={0xc0,0xf9,0xa4,0xb0,0x99,0x92,0x82,0xf8,0x80,0x90,0x88,0x83,0xc6,0xa1,0x86,0x8e};    // ma hien thi LED 7 doan
//               0    1    2     3    4    5   6    7    8     9    a    b    c    d    e    f
//***************cac chuong trinh con********************
// phat hien bat ki phim nao duoc nhan
// tra ve 1 : co nhan phim, 0: ko co phim nao duoc nhan

unsigned char KeyPressed()
{
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);
    HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,GPIO_PIN_RESET);
    // row0=row1=row2=row3=0;
    if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==0||HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==0||HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5)==0)
    {
        return 1;
    }
    return 0;
}

// phat hien vi tri Cot duoc nhan 
unsigned char DetectCol()
{
  unsigned char c=0;
  if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3))c=1;
  else if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4))c=2;
  else if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5))c=3;
  return c;
}

// Cho hang tuong ung xuong muc thap cac hang con lai len muc cao
void  ScanRow(int row)
{
 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,1);
 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,1);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,1);
 // row0=row1=row2=row3=1;
  if(row==0)     HAL_GPIO_WritePin(GPIOD,GPIO_PIN_4,0);// row0=0;
  else if(row==1) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_5,0);// row1=0;
  else if(row==2) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_6,0);// row2=0;
  else if(row==3) HAL_GPIO_WritePin(GPIOD,GPIO_PIN_7,0);// row3=0;
}

// Tinh toan vi tri phim (0- 16 <=> 0....f)
unsigned char KeyCode()
{
  unsigned char i,Col;
  for(i=0;i<4;i++)
  {
    ScanRow(i);
    Col=DetectCol();
    if(Col>0) break; 
  }
    return (i*4 + Col-1);
}

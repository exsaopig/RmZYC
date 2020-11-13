#include "bsp_imu_9250.h"

void IIC_Init(void)
{
	SDA_OUT();
	IIC_SDA_UP();
	IIC_SCL_UP();
	
}

void IIC_Start(void)
{
	SDA_OUT();
	IIC_SDA_UP();
	IIC_SCL_UP();
	for(int i=0;i<150;i++)  
	;
	IIC_SDA_DOWN();
	for(int i=0;i<160;i++)  
	;
		IIC_SCL_DOWN();
}

void IIC_Stop(void)
{
	IIC_SCL_DOWN();
	IIC_SDA_DOWN();
	for(int i=0;i<100;i++) 
	;
	IIC_SCL_UP();
	IIC_SDA_UP();
}
uint8_t IIC_Wait_Ack(void)
{
		uint8_t ucErrTime=0; 
		SDA_IN();   
		IIC_SDA_UP();
	 for(int i=0;i<100;i++) 
	 ;
	  while(IIC_SDA_READ())
		{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 1;
		}
	 for(int i=0;i<100;i++) 
		;
		}
	IIC_SCL_UP();
	 for(int i=0;i<100;i++) 
    ;
	IIC_SCL_DOWN();//时钟输出0  
	return 0;  
}
void IIC_Ack(void)
{
	IIC_SCL_DOWN();
	SDA_OUT();
	IIC_SDA_DOWN();
	 for(int i=0;i<100;i++) 
	 ;
	IIC_SCL_UP();
	 for(int i=0;i<100;i++) 
	 ;
	IIC_SCL_DOWN();
}
void IIC_NAck(void)
{
	IIC_SCL_DOWN();
	SDA_OUT();
	IIC_SDA_UP();
	for(int i=0;i<100;i++) 
  ;
	IIC_SCL_UP();
	for(int i=0;i<100;i++) 
   ;
	IIC_SCL_DOWN();
}					 				     

void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t; 
		SDA_OUT(); 	    
    IIC_SCL_DOWN();
    for(t=0;t<8;t++)
    {              
			if((txd&0x80)>>7==1)
					IIC_SDA_UP();
				else
					IIC_SDA_DOWN();
			
				txd<<=1; 
	 for(int i=0;i<40;i++)
		; //2us
				IIC_SCL_UP();
	 for(int i=0;i<100;i++) 
		; //5us
				IIC_SCL_DOWN();	
	 for(int i=0;i<60;i++) 
		; //3us
    }	 
}
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
     IIC_SCL_DOWN();  
	 for(int i=0;i<160;i++) 
		;
		 IIC_SCL_UP();
     receive<<=1;
     if(IIC_SDA_READ())receive++;   
	 for(int i=0;i<160;i++) 
   ;
	}					 
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
}

uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
    uint8_t count = 0;
	
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
  IIC_Wait_Ack();	  
	IIC_Start();
	IIC_Send_Byte((dev<<1)+1);  //进入接收模式	
	IIC_Wait_Ack();
    for(count=0;count<length;count++){
		 
		 if(count!=length-1)data[count]=IIC_Read_Byte(1);  //带ACK的读数据
		 	else  data[count]=IIC_Read_Byte(0);	 //最后一个字节NACK
	}
    IIC_Stop();//产生一个停止条件
	  IIC_Stop();
	  IIC_Stop();
    return count;
	
}
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data){
  
 	uint8_t count = 0;
	IIC_Start();
	IIC_Send_Byte(dev<<1);	   //发送写命令
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);   //发送地址
	IIC_Wait_Ack();	  
	for(count=0;count<length;count++){
		IIC_Send_Byte(data[count]); 
		IIC_Wait_Ack(); 
 }
	IIC_Stop();//产生一个停止条件

    return 1; //status == 0;
	
}




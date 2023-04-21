#include <atmel_start.h>
#include "hal_can_async.h"

typedef struct{
	float DCDC_clock;
	float voltage;
	float power;
	float water_temp;
	float ava_current;
}ParamList;

ParamList paramList;

uint8_t modReceiveFlag = 0;
uint8_t modTaskTrigger = 0;
static uint8_t mctr = 0;

void CAN_0_tx_callback1(struct can_async_descriptor *const descr)
{
	(void)descr;
}
uint8_t arr1[8];
uint8_t arr2[8];
uint8_t arr3[8];
uint32_t cnt[4];



void dataDecode(void){

	int DCDC_clock = (arr3[4] & 0xF0) >> 4;
	int ava_current = arr3[0];
	int water_temp = ((arr2[0] & 0x1F) << 3) | ((arr2[1] & 0xE0) >> 5 );
	int power = (arr1[3] & 0x7E) >> 1;
	int voltage = ((arr1[1] & 0x0F) << 9 ) | (arr1[2] << 1 ) | (arr1[3] & 0x80) >> 7;

	paramList.DCDC_clock =  (DCDC_clock * 1);
	paramList.voltage = (float) ((voltage * 0.1) + 0);
	paramList.power = (float) ((power * 0.1) + 0);
	paramList.water_temp = (float) ((water_temp * 1) - 40);
	paramList.ava_current = (float) ((ava_current * 1) + 0);
}

void CAN_0_rx_callback1(struct can_async_descriptor *const descr)
{
	struct can_message msg;
	uint8_t data[64] = {0};
	memset(&msg,0,sizeof(msg));
	msg.data = data;
	can_async_read(descr, &msg);
	if(msg.id==0x71){
		memcpy(arr1,msg.data,msg.len);
		cnt[0]++;
	}else if(msg.id==0x62C){
		memcpy(arr2,msg.data,msg.len);
		cnt[1]++;
	}else if(msg.id==0x45E){
	memcpy(arr3,msg.data,msg.len);
		cnt[2]++;
	}else{
		cnt[3]++;
	}
	return;
}

void CAN_0_irq_callback1(struct can_async_descriptor *const canVar, enum can_async_interrupt_type intType)
{
	int a = intType;
	return;
}


/**
 * Example of using CAN_0 to Encrypt/Decrypt datas.
 */
void CAN_0_example1(void)
{
	struct can_message msg;
	struct can_filter  filter;
	uint8_t            send_data[4];
	send_data[0] = 0x12;
	send_data[1] = 0x01;
	send_data[2] = 0x02;
	send_data[3] = 0x03;

	msg.id   = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback1);
	//can_async_register_callback(&CAN_0, CAN_ASYNC_IRQ_CB, (FUNC_PTR)CAN_0_irq_callback1);
	can_async_enable(&CAN_0);

	/**
	 * CAN_0_tx_callback callback should be invoked after call
	 * can_async_write, and remote device should recieve message with ID=0x45A
	 */
	can_async_write(&CAN_0, &msg);

	msg.id  = 0x100000A5;
	msg.fmt = CAN_FMT_STDID;
	/**
	 * remote device should recieve message with ID=0x100000A5
	 */
	can_async_write(&CAN_0, &msg);

	/**
	 * CAN_0_rx_callback callback should be invoked after call
	 * can_async_set_filter and remote device send CAN Message with the same
	 * content as the filter.
	 */
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback1);
	filter.id   = 0x469;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);

	filter.id   = 0x10000096;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);
}
struct io_descriptor *iox;
static void tx_cb_USART_0X(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}
static void rx_cb_USART_0X(const struct usart_async_descriptor *const io_descr)
{
	modReceiveFlag = 1;/* Transfer completed */
}

static void err_cb_USART_0X(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

uint16_t ModRTU_CRC(uint8_t* buf, int len)
{
	uint16_t crc = 0xFFFF;
	
	for (int pos = 0; pos < len; pos++) {
		crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
		
		for (int i = 8; i != 0; i--) {    // Loop over each bit
			if ((crc & 0x0001) != 0) {      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			}
			else                            // Else LSB is not set
			crc >>= 1;                    // Just shift right
		}
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
	return crc;
}

void modbusTask(){
	uint8_t rxBuf[64];
	io_read(iox,&rxBuf[0],8);
	int i;
	uint16_t crc;
	uint8_t stdMdReq[8] = {0x01,0x03,0x00,0x00,0x00,0x0A,0xC5,0xCD};
	uint8_t stdMdRsp[25] = {0x01,0x03,0x14};
	if(memcmp(stdMdReq,rxBuf,8)!=0){
		return;
	}
	uint8_t * ptr =(uint8_t*) &paramList;
	for(i=0;i<10;i++){
		stdMdRsp[i*2+3] = ptr[i*2 + 1];
		stdMdRsp[i*2+4] = ptr[i*2];
	}
	crc = ModRTU_CRC(stdMdRsp,23);
	stdMdRsp[23] = crc&0xFF;
	stdMdRsp[24] = crc>>8;
	io_write(iox,stdMdRsp,25); 
}

static uint8_t example_USART_0X[12] = "Hello World!";
void USART_0_example_init(void)
{
	usart_async_register_callback(&USART_0, USART_ASYNC_TXC_CB, tx_cb_USART_0X);
	usart_async_register_callback(&USART_0, USART_ASYNC_RXC_CB, rx_cb_USART_0X);
	usart_async_register_callback(&USART_0, USART_ASYNC_ERROR_CB, err_cb_USART_0X);
	usart_async_get_io_descriptor(&USART_0, &iox);
	usart_async_enable(&USART_0);


}

void uart_routine(){
		io_write(iox, example_USART_0X, 12);
}

void can_send_test(void){
	struct can_message msg;
	struct can_filter  filter;
	static uint8_t k = 0;
	
	k++;

	
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback1);
	can_async_enable(&CAN_0);

	/**
	 * CAN_0_tx_callback callback should be invoked after call
	 * can_async_write, and remote device should recieve message with ID=0x45A
	 */
	can_async_write(&CAN_0, &msg);

	//msg.id  = 0x100000A5;
	//msg.fmt = CAN_FMT_EXTID;
	/**
	 * remote device should recieve message with ID=0x100000A5
	 */
	//can_async_write(&CAN_0, &msg);

	
}

int err,err1;
uint8_t   dataX[64];
int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback1);
	uint8_t            send_data[8];
	send_data[0] = 0xC0;
	send_data[1] = 0x40;
	send_data[2] = 0x00;
	send_data[3] = 0x00;
	send_data[4] = 0x00;
	send_data[5] = 0xE0;
	send_data[6] = 0x00;
	send_data[7] = 0x0F;
	struct can_filter  filter;
	//filter.id   = 0b10101010101;
	filter.id   = 0x0;
	filter.mask = 0x980;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);
	can_async_enable(&CAN_0);
	struct can_message msg;
	msg.type = CAN_TYPE_DATA;
	//msg.type = CAN_TYPE_REMOTE;
	msg.id   = 0x430;
	msg.data = send_data;
	msg.len  = 8;
	msg.fmt  = CAN_FMT_STDID;
	USART_0_example_init();
	/* Replace with your application code */
	while (1) {
		//can_send_test();
		if(modReceiveFlag==1){
			modReceiveFlag = 0;
			modTaskTrigger = 1;
		}
		delay_ms(100);
		can_async_write(&CAN_0, &msg);
		//uart_routine();
		err = can_async_get_txerr(&CAN_0);
		err1 = can_async_get_rxerr(&CAN_0);
		dataDecode();
		
		 if(modTaskTrigger==1){
			 mctr++;
			 if(mctr>3){
				 mctr=0;
				 modbusTask();
				 modTaskTrigger = 0;
			 }
	}
}
}


#include "wdt_dio.h"
#include <unistd.h>
#include <stdio.h>

int can()
{
	printf("Running\n");
		CAN_SETUP setup;

		setup.bitRate = 250000;//25 kbps
		printf("channel = %i; _bitRate = %i\n", 0, setup.bitRate);
		if (setup.bitRate == 0)
		{
			printf("ERROR: Unsupported bit rate\n");
			return -1;
		}
		
		setup.recvConfig = CAN_MSG_USE_ID_FILTER;
		setup.recvId = 0;
		setup.recvMask = 0;

		if (!CAN_Setup(0, &setup, sizeof(setup)))
		{
			printf("ERROR: CAN_Setup() failed\n");
			return -1;
		}

		if (!CAN_Start(0))
		{
			printf("ERROR: CAN_Start() failed\n");
			return -1;
		}
		
		CAN_MSG  txMsg;
		//memset(&txMsg, 0, sizeof(txMsg));
		txMsg.id = 0x1A0;
		//txMsg.flags = CAN_MSG_EXTENDED_ID;
		txMsg.len = 8;

		for (int i=0; txMsg.len > i; ++i )
		{
			txMsg.data[i] = 0x33;
		}	

		if ( ! CAN_Send(0, &txMsg, sizeof(txMsg)) )
		{
			printf("CAN_MessageSet --> FAILED\n");
			return -1;
		}
		
		CAN_Stop(0);
		printf("Success!!\n");
}



int square(int i){
	return i*i;
}
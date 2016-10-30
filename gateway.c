#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <time.h>


#include "rfm69.h"
#include "rfm69config.h"
#include "nodeconfig.h"

void setupRFM69()
{  
  
	if (!rfm69_init(1))
	{
		printf("RFM69 Failed\n");
		exit(1);
	}
	else
	{
		printf("RFM69 Booted\n");
	}
  
}

int main(int argc, char **argv)
{
	char Message[65];
	char seq = 97;

	printf("**** UKHASNet Pi Gateway by daveake ****\n");
	
	// put the gateway on the map
	snprintf(Message,60,"0aL%s[%s]", LOCATION_STRING, NODE_ID);
	printf("%s\n", Message);
//	UploadPacket(Message,0);

	setupRFM69();

	while (1)
	{
		printf("%s\n", Message);
		RFM69_tx(Message);
		Message[1] = seq++;
		if (seq > 122)
			seq = 98;
		sleep(10);
	}

	return 0;
}


/* tool to read power production data for SMA solar power convertors */
/* Written by Wim Hofman*/
/* compile gcc -lbluetooth -lcurl -lmysqlclient -o smatool smatool.c */


#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/types.h>
#include <curl/curl.h>
#include "mysql.c"

typedef unsigned short u16;
#define PPPINITFCS16 0xffff /* Initial FCS value    */
#define PPPGOODFCS16 0xf0b8 /* Good final FCS value */

char *accepted_strings[] = {
"$END",
"$ADDR",
"$TIME",
"$SER",
"$CRC",
"$POW",
"$DTOT",
"$ADD2",
"$CHAN"
};

int cc,verbose = 0;
unsigned char fl[1024] = { 0 };



static u16 fcstab[256] = {
0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/*
 * Calculate a new fcs given the current fcs and the new data.
 */
u16 pppfcs16(fcs, cp, len)
    register u16 fcs;
    register unsigned char *cp;
    register int len;
{
    //ASSERT(sizeof (u16) == 2);
  //ASSERT(((u16) -1) > 0);
	while (len--)
      fcs = (fcs >> 8) ^ fcstab[(fcs ^ *cp++) & 0xff];
  return (fcs);
}

/*
 * How to use the fcs
 */
tryfcs16(cp, len)
    register unsigned char *cp;
    register int len;
{
    u16 trialfcs;
    int i;

    /* add on output */
    if (verbose ==1){
	 	printf("String to calculate FCS\n");
	 	for (i=0;i<len;i++) printf("%02X ",cp[i]);
	 	printf("\n\n");
	 }
    trialfcs = pppfcs16( PPPINITFCS16, cp, len );
    trialfcs ^= 0xffff;               /* complement */
    fl[cc] = (trialfcs & 0x00ff);    /* least significant byte first */
    fl[cc+1] = ((trialfcs >> 8) & 0x00ff);
	 cc = cc + 2;
	 if (verbose == 1) printf("FCS = %x%x\n",(trialfcs & 0x00ff),((trialfcs >> 8) & 0x00ff));
}


unsigned char conv(char *nn){
	unsigned char tt=0,res=0;
	int i;

	for(i=0;i<2;i++){
		switch(nn[i]){

		case 65:
		tt = 10;
		break;

		case 66:
		tt = 11;
		break;

		case 67:
		tt = 12;
		break;

		case 68:
		tt = 13;
		break;

		case 69:
		tt = 14;
		break;

		case 70:
		tt = 15;
		break;


		default:
		tt = nn[i] - 48;
		}
		res = res + (tt * pow(16,1-i));
		}
		return res;
}

int select_str(char *s)
{
int i;
for (i=0; i < sizeof(accepted_strings)/sizeof(*accepted_strings);i++)
if (!strcmp(s, accepted_strings[i])) return i;
return -1;
}

int main(int argc, char **argv)
{
	FILE *fp;
	struct sockaddr_rc addr = { 0 };
	unsigned char buf[1024];
	unsigned char received[1024];
	int bytes_read,s,i,status,mysql=0,post=0;
	int ret,found=0;
	char dest[18];
	char ser[18];
	char url[100];
	char compurl[200];
	char line[400];
	char address[6] = { 0 };
	char address2[6] = { 0 };
	char serial[4] = { 0 };
	char *lineread;
	time_t curtime;
	struct tm *loctime;
	int day,month,year,hour,minute,datapoint;
	char tt[10] = {48,48,48,48,48,48,48,48,48,48};
	char ti[3];
	char pac[2];
	char tot[2];
	char chan[1];
	int currentpower,rr;
	float dtotal;
	struct timeval tv;
	fd_set readfds;
	MYSQL_ROW row;
	char SQLQUERY[200];
	char rowres[10];

   char *server = "localhost";
   char user[100];
   char password[100];
   char *database = "smatool";

   CURL *curl;
   CURLcode result;

   memset(buf,0,1024);
   memset(received,0,1024);

     for (i=1;i<argc;i++)			//Read through passed arguments
      {
				if (strcmp(argv[i],"-address")==0){
                  i++;
                  if (i<argc){
							strcpy(dest,argv[i]);
                 }
				}
				if (strcmp(argv[i],"-serial")==0){
                  i++;
                  if (i<argc){
							strcpy(ser,argv[i]);
                 }
				}
				if (strcmp(argv[i],"-u")==0){
                  i++;
                  if (i<argc){
							strcpy(user,argv[i]);
                 }
            }
				if (strcmp(argv[i],"-p")==0){
                  i++;
                  if (i<argc){
							strcpy(password,argv[i]);
                 }
				}
				if (strcmp(argv[i],"-v")==0) verbose = 1;
				if (strcmp(argv[i],"-mysql")==0) mysql=1;
				if (strcmp(argv[i],"-post")==0){
					i++;
					if(i<argc){
						strcpy(url,argv[i]);
						post = 1;
					}
				}
		}

	if (verbose ==1) printf("Adress %s\n",dest);

	fp=fopen("/etc/sma.in","r");
	// allocate a socket
   s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

   // set the connection parameters (who to connect to)
   addr.rc_family = AF_BLUETOOTH;
   addr.rc_channel = (uint8_t) 1;
   str2ba( dest, &addr.rc_bdaddr );

   // connect to server
   status = connect(s, (struct sockaddr *)&addr, sizeof(addr));
	if (status <0){
		printf("Error connecting to %s\n",dest);
		return -1;
	}




	// convert address
	address[5] = conv(strtok(dest,":"));
	address[4] = conv(strtok(NULL,":"));
	address[3] = conv(strtok(NULL,":"));
	address[2] = conv(strtok(NULL,":"));
	address[1] = conv(strtok(NULL,":"));
	address[0] = conv(strtok(NULL,":"));

	// convert serial
	serial[3] = conv(strtok(ser,":"));
	serial[2] = conv(strtok(NULL,":"));
	serial[1] = conv(strtok(NULL,":"));
	serial[0] = conv(strtok(NULL,":"));

while (!feof(fp)){
	if (fgets(line,400,fp) != NULL){				//read line from sma.in
		lineread = strtok(line," ;");
		if(!strcmp(lineread,"R")){		//See if line is something we need to receive
			if (verbose	== 1) printf("Waiting for string\n");
			cc = 0;
			do{
				lineread = strtok(NULL," ;");
				switch(select_str(lineread)) {

				case 0: // $END
				//do nothing
				break;

				case 1: // $ADDR
				for (i=0;i<6;i++){
					fl[cc] = address[i];
					cc++;
				}
				break;

				case 7: // $ADD2
				for (i=0;i<6;i++){
					fl[cc] = address2[i];
					cc++;
				}
				break;

				case 3: // $SER
				for (i=0;i<4;i++){
					fl[cc] = serial[i];
					cc++;
				}
				break;

				case 8: // $CHAN
				fl[cc] = chan[0];
				cc++;
				break;

				default :
				fl[cc] = conv(lineread);
				cc++;
				}

			} while (strcmp(lineread,"$END"));
			if (verbose == 1){
				for (i=0;i<cc;i++) printf("%02X ",fl[i]);
			   printf("\n\n");
			}
			if (verbose == 1) printf("Waiting for data on rfcomm\n");
			found = 0;
			do {
				tv.tv_sec = 5; // set timeout of reading to 5 seconds
				tv.tv_usec = 0;

				FD_ZERO(&readfds);
				FD_SET(s, &readfds);

				select(s+1, &readfds, NULL, NULL, &tv);

				if (FD_ISSET(s, &readfds)){	// did we receive anything within 5 seconds
					bytes_read = recv(s, buf, sizeof(buf), 0);
				}
				else
				{
					printf("Timeout reading bluetooth socket\n");
					return -1;
				}
				if ( bytes_read > 0){
					if (verbose == 1) printf("Data received on rfcomm\n");
					rr = 0;
					for (i=0;i<bytes_read;i++){ //start copy the rec buffer in to received
						if (buf[i] == 0x7d){ //did we receive the escape char
							switch (buf[i+1]){   // act depending on the char after the escape char

							case 0x5e :
							received[rr] = 0x7e;
							break;

							case 0x5d :
							received[rr] = 0x7d;
							break;

							default :
							received[rr] = buf[i+1] ^ 0x20;
							break;
							}
							i++;
						}
						else received[rr] = buf[i];
						if (verbose == 1) printf("%02X ", received[rr]);
						rr++;
					}
					if (verbose == 1) printf("\n\n");
				}
				if (memcmp(fl,received,cc) == 0){
					found = 1;
					if (verbose == 1) printf("Found string we are waiting for\n\n");
				}
			}	while (found == 0);
		}
		if(!strcmp(lineread,"S")){		//See if line is something we need to send
			if (verbose	== 1) printf("Sending string\n");
			cc = 0;
			do{
				lineread = strtok(NULL," ;");
				switch(select_str(lineread)) {

				case 0: // $END
				//do nothing
				break;

				case 1: // $ADDR
				for (i=0;i<6;i++){
					fl[cc] = address[i];
					cc++;
				}
				break;

				case 7: // $ADD2
				for (i=0;i<6;i++){
					fl[cc] = address2[i];
					cc++;
				}
				break;

				case 2: // $TIME
				// get unix time and convert
				curtime = time(NULL);  //get time in seconds since epoch (1/1/1970)
				sprintf(tt,"%X",(int)curtime); //convert to a hex in a string
				for (i=9;i>0;i=i-2){ //change order and convert to integer
					ti[1] = tt[i];
					ti[0] = tt[i-1];
					ti[2] = 0;
					fl[cc] = conv(ti);
					cc++;
				}
				break;

				case 4: //$crc
				tryfcs16(fl+19, cc -19);
				break;

				case 8: // $CHAN
				fl[cc] = chan[0];
				cc++;
				break;


				default :
				fl[cc] = conv(lineread);
				cc++;
				}

			} while (strcmp(lineread,"$END"));
			if (verbose == 1){
				for (i=0;i<cc;i++) printf("%02X ",fl[i]);
			   printf("\n\n");
			}
			write(s,fl,cc);
		}


		if(!strcmp(lineread,"E")){		//See if line is something we need to extract
			if (verbose	== 1) printf("Extracting\n");
			cc = 0;
			do{
				lineread = strtok(NULL," ;");
				switch(select_str(lineread)) {

				case 5: // extract current power
				memcpy(pac,received+67,2);
				currentpower = (pac[1] * 256) + pac[0];
				printf("Current power = %i Watt\n",currentpower);
				break;

				case 6: // extract total energy collected today
				memcpy(tot,received+83,2);
				dtotal = (tot[1] * 256) + tot[0];
				dtotal = dtotal / 1000;
            printf("E total today = %.2f Kwh\n",dtotal);
            break;

				case 7: // extract 2nd address
				memcpy(address2,received+26,6);
				if (verbose == 1) printf("address 2 \n");
				break;

				case 8: // extract bluetooth channel
				memcpy(chan,received+22,1);
				if (verbose == 1) printf("Bluetooth channel = %i\n",chan[0]);
				break;

				}

			} while (strcmp(lineread,"$END"));
		}
	}
}

loctime = localtime(&curtime);
day = loctime->tm_mday;
month = loctime->tm_mon +1;
year = loctime->tm_year + 1900;
hour = loctime->tm_hour;
minute = loctime->tm_min;
datapoint = (int)(((hour * 60) + minute)) / 5;

if (post ==1){
	ret=sprintf(compurl,"%s?date=%i-%i-%i&time=%i:%i:00&currentpower=%i&dtotal=%.2f&datapoint=%i&user=%s&password=%s",url,year,month,day,hour,minute,currentpower,dtotal,datapoint,user,password);
	if (verbose == 1) printf("url = %s\n",compurl);
	curl = curl_easy_init();
	if (curl){
		curl_easy_setopt(curl, CURLOPT_URL, compurl);
		result = curl_easy_perform(curl);
		curl_easy_cleanup(curl);
	}
}


if (mysql ==1){
	/* Connect to database */
   OpenMySqlDatabase( server, user, password, database);

	sprintf(SQLQUERY,"SELECT CurrentPower, Time FROM DayData WHERE Date = '%i-%02i-%02i' AND DataPoint = %i ORDER BY Time DESC LIMIT 1",year, month,day,datapoint);
	if (verbose == 1) printf("%s\n",SQLQUERY);
	DoQuery(SQLQUERY);
	if (row = mysql_fetch_row(res)){  //if there is a result, update the row
		sprintf(rowres,"%s\n",row[0]);
		currentpower = (int) (currentpower + atoi(rowres)) / 2;
		ret = sprintf(SQLQUERY,"UPDATE DayData SET Time = '%02i:%02i:00' , CurrentPower = %i , ETotalToday = %.2f WHERE Date = '%i-%02i-%02i' AND DataPoint = %i",hour,minute,currentpower,dtotal,year,month,day,datapoint);
		if (verbose == 1) printf("%s\n",SQLQUERY);
		DoQuery(SQLQUERY);
	}
	else {
		ret = sprintf(SQLQUERY,"INSERT INTO DayData (Date, Time, CurrentPower,ETotalToday,DataPoint) VALUES ('%02i-%02i-%02i','%02i:%02i:00',%i,%.2f,%i)",year,month,day,hour,minute,currentpower,dtotal,datapoint);
		if (verbose == 1) printf("%s\n",SQLQUERY);
		DoQuery(SQLQUERY);
	}

	mysql_close(conn);
}
close(s);
return 0;
}

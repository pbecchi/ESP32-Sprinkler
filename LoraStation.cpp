
#include "LoraStation.h"

#include <SPI.h>

#ifndef ESP32
#include<avr/eeprom.h>
static RH_RF95 rf95;
#else
#include "Adafruit_SSD1306.h"
#include "Pins.h"
#include <EEPROM.h>
RH_RF95 rf95(SPI_CS, LORA_IRQ);

#include "OpenSprinkler.h"

#include "Eeprom_ESP.h"

#endif
//#define EE_ADDRESS_RSSI (byte *)2301
//#define EE_ADDRESS_N_NODES (byte *)2300
#define N_NODES LORA_MAX_STA //max number of nodes
#define N_LOOPS 5

#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)


#define INTERVAL 2000L
byte mycode = 0;
long syncro = 0, mytime = 0;
byte asknode = 1;                         //<--------------identifcatore del nodo

//bool scanmode = false;
byte RSSI[N_NODES];


byte node[N_LOOPS];   //nodes of the path pathto return n.of nodes;
#define NLEVELS 4
long mask[NLEVELS][N_NODES];

void LoraStation::LoraSend(String s) {
	rf95.setHeaderFrom(mycode);
	rf95.setHeaderTo(LoraTo);
	rf95.send((byte *)s.c_str(), s.length());
	rf95.waitPacketSent();
}
void LoraStation::LoraSend(char* s) {
	rf95.setHeaderFrom(mycode);
	rf95.setHeaderTo(LoraTo);
	rf95.send((byte *)s, strlen(s));
	rf95.waitPacketSent();
}

void LoraStation::LoraSend(byte add, char* s) {
	char buf[250]="";
	if (routing[add] == 0) {
		Serial.print(add);
		rf95.setHeaderTo(add);
		strcpy(buf, s);
	} else {
		long code = routing[add];
		rf95.setHeaderTo(code & 0xFF);
		Serial.print(code & 0xFF);
			while (code != 0) {
				char address[6];
				sprintf(address, "/%d", code & 0xFF);
				strcat(buf, address);
				code = code >> 8;
			}
			strcat(buf, s);
		
	}
	Serial.println(buf);
	rf95.setHeaderFrom(mycode);
	Serial.println(" Send..");
	rf95.send((byte *)buf, strlen(buf));
	rf95.waitPacketSent();
}
void strshorten(char * buf, byte ii) {
	byte i;
	for ( i = ii; i < strlen(buf); i++)
		buf[i - ii] = buf[i];
	buf[i - ii + 1] = 0;
}
byte LoraStation::readLora(byte * buf, uint8_t len) { // read packet and send it to next recipient

	if (rf95.available()) {
		Serial.print('_');
		// Should be a reply message for us now   
		if (rf95.recv(buf, &len)) {
			LoraFrom = rf95.headerFrom();
			Serial.print("got reply: ");
			Serial.println((char*)buf);
			Serial.print("RSSI: ");
			Serial.println(rf95.lastRssi(), DEC);
			///////////////////    _________________________send to next recipient_______________
			buf[len] = 0;
			char * cc; byte add[5], i = 0, ii = 0;
			if (strchr((char *)buf + 1, '/') != NULL) {
				char buff[40];
				strcpy(buff, (char *)buf);
				cc = strtok((char *)buf + 1, "/");
				while (cc != NULL) {
					Serial.println(cc);
					ii += strlen(cc) + 1;
					add[i++] = atoi(cc);
					cc = strtok(NULL, "/");
				}
				//			Serial.println(add[i - 2]);
				if (add[i - 2] != mycode) {
					for (byte j = i - 2; j >= 0; j--) {
						//				Serial.println(add[j]);
						if (add[j] == mycode) {
							LoraTo = add[j + 1];
							LoraSend((char*)buff);
							Serial.print(F("retransmitted to")); Serial.println(LoraTo);
							//	LoraTo = 0;
							//	LoraSend("retrasmitted");
							return 0;
						}
					}
				} else {
					Serial.println(F("received"));
					strshorten((char *)buf, ii);
					len = len - ii;
				}
			}

			return len;
		}

		/* add main program code here */
		return 0;
	}
	return 0;
}


byte inde(byte rw, byte cl) {
	int row = rw, col = cl, res;
	if (col == row)return 0;
	if (col > row)
		res = (col - 1)*(col - 1) - ((col - 2)*(col - 1)) / 2 + row;
	else
		res = (row - 1)*(row - 1) - ((row - 2)*(row - 1)) / 2 + col;
	if (res < 0)return 0;
	return res;
}
#define LEV_MAX 120
#define LEV_MIN 60
void maskRSSI(byte imax, byte * RSSItot) {
	for (byte i = 0; i < imax; i++) {
		int dlevel = (LEV_MAX-LEV_MIN) / NLEVELS;
		for (byte l = 0; l < NLEVELS; l++) {
			int max = LEV_MAX - l*dlevel;
			mask[l][i] = 0;
			
			for (byte j = 0; j < imax; j++) if (i != j) {
				if (RSSItot[inde(i, j)] < max && RSSItot[inde(i, j)]>0) {
					mask[l][i] += 1 << j;
				}
				Serial.print(RSSItot[inde(i, j)]); Serial.print(' ');
			}
			Serial.println(mask[l][i], BIN);

		}
	}
}
byte  findmatch(byte &node1, byte&node2, byte& node3) {//find node2 that connect 2 nodes at minimum RSSI retrun RSSI level or 0           
	int lev;
	Serial.print(node1);Serial.print(' ');Serial.println(node3);
	for (lev = NLEVELS-1; lev >= 0; lev--) {                          //test if connected from low level increasing
	//	Serial.println(mask[lev][node1]& mask[lev][node3]);
		if ((mask[lev][node1] & mask[lev][node3]) != 0)	break;     ///there is  connection at this level!
	}
	
	//Serial.println(lev);
	
	if (lev < 0) return 0;                                                         /// no connection at any level

	    Serial.print(F("lev=")); Serial.println(lev);
		Serial.println(mask[lev][node1], BIN);
		Serial.println(mask[lev][node3], BIN);

		byte j;                                                                     ///ther is connection
		for (j = 0; j < N_NODES; j++) {                                             ///find connected nodes
			//Serial.println(((mask[lev][node1] & mask[lev][node3]) &(1 << j)));
			if (((mask[lev][node1] & mask[lev][node3]) &(1 << j))!=0)break;         ///node j is connected to node1

		}
		//no 
		node2 = j;
		Serial.println(node2);
		return lev+1;

}
byte connect(byte lev, byte con, byte node) {
	return mask[lev][node] >> con & 1;                ///node j is connected to node1


}
//----------------------path algoritm------------------------------------
byte pathTo(byte thisnode, byte lastnode,int lev=0) {
	//loop on this node receivers
	
		while (lev-- >= 0) {
			node[1] = thisnode; node[3] = lastnode;
			if (findmatch(node[1], node[2], node[3]) > lev) return 3;
			node[4] = node[3];
			//for (byte i=0;i<N_LOOPS;i++){
			Serial.println(F("LEVEL2"));
		//	for (byte lev = 1; lev < NLEVELS; lev++)
				for (byte j = 1; j < N_NODES; j++)
					if (connect(lev, j, node[1])) {
						node[2] = j;
						if (findmatch(node[2], node[3], node[4]) > lev) return 4;
					}
			node[5] = lastnode;
			Serial.println(F("LEVEL3"));
		//	for (byte lev = 1; lev < NLEVELS; lev++)

				for (byte j = 1; j < N_NODES; j++)
					if (connect(lev, j, node[5])) {
						node[4] = j;
						for (byte jj = 1; jj < N_NODES; jj++)
							if (connect(lev, jj, node[1])) {
								node[2] = jj;
								if (findmatch(node[2], node[3], node[4]) > lev) return  5;
							}


					}
		}
	return 0;
}


LoraStation::LoraStation() {
}



LoraStation::~LoraStation() {
}

#define LIMIT_RSSI 110
#define nTimes 3
bool LoraStation::begin(byte cadmode=false) {
	if (!rf95.init()) {
		Serial.println("init failed");
		return false;
	}
	rf95.setTxPower(20);
#ifdef LORA915
	rf95.setFrequency(915.0);
#endif  //----------------------------------------default is 433-------------------
	//rf95.setPreambleLength(500);
	rf95.setThisAddress(0);
	rf95.setTxPower(20);
	if (cadmode) {
		rf95.setPreambleLength(500);
		rf95.waitCAD();
	}

}
OpenSprinkler oss;

bool LoraStation::findRouting(byte n_nodes, bool find_node_routing,byte allNodes=0){   // byte allNodes = 0,byte n) {
	byte RSSItot[(N_NODES*(N_NODES - 1)) / 2 + 1];
	//if allNodes=0 will rescan all Lora terminal in the area
	// if allNodes=n_nodes  read from eeprom N_nodes and RSSI matrix and define routing to nodes
	byte totn = 0;
	if (find_node_routing == true) {

		oss.lcd.clearDisplay();
		oss.lcd.setCursor(0, 0);
		oss.lcd.printf("Scanning %d LoRa Nodes...press Boot to exit", n_nodes);
		oss.lcd.display();

		Serial.println(F("Scanning all Lora nodes this may take up to 10 min."));
		unsigned long maxMilli = millis() + 60000L*n_nodes;
		bool scanmode = true;
		while (allNodes < n_nodes&&millis() < maxMilli&&digitalRead(0)==1) {

			byte buf[100]; byte len = 100;
			byte nNodes = n_nodes;
			//Serial.print('.');
			//------------------read incoming messages---------------------------
			//------only------------ scan or RSSIres ----------------------------
			if (readLora(buf, len)) {
				char* str;
				//Serial.print('_');
				str = strtok((char*)buf, ",\0");
				Serial.println(str);
        if(str!=NULL)
				if (strcmp(str, "scan") == 0) {
					if (rf95.headerTo() == mycode) {
						str = strtok(NULL, ",\0");
				//		if (nNodes == 0)nNodes = atoi(str);
						Serial.println(nNodes);
						//syncro = 0;

					/*	if (notscanned) {
							scanmode = true; notscanned = false;
							asknode = rf95.headerFrom();          ////read scan command
							if (mycode>asknode)
								mytime = (mycode - asknode) *nNodes* INTERVAL + millis();
							else
								mytime = (asknode - mycode) *nNodes* INTERVAL + millis() + nNodes*nNodes*INTERVAL;

						}*/
						char nn[12]; char  mess[16] = "RSSI,";    ////send replay with RSSI
						strcat(mess, itoa(-rf95.lastRssi(), nn, 10));
						LoraTo = rf95.headerFrom();
						LoraSend(mess);
						Serial.print(F("scan request from")); Serial.println(rf95.headerFrom());
					}
				} else
					if (strcmp(str, "RSSIres") == 0) {
						if (mycode == 0) {                  /////read  RSSI results
							str = strtok(NULL, ",\0");
							LoraFrom = atoi(str);
							for (byte i = 0; i < nNodes; i++) {
								str = strtok(NULL, ",\0");
								if (str!=NULL&&i != LoraFrom) {
						
									RSSItot[inde(i, LoraFrom)] = atoi(str);
									Serial.print(inde(i, LoraFrom)); Serial.print(' '); Serial.println(RSSItot[inde(i, LoraFrom)]);
								}
							}

							oss.lcd.setCursor(0, 10);
							oss.lcd.printf("Found node n.%d ,total %d found", LoraFrom, totn++);
							oss.lcd.display();
							Serial.print(F("RSSItot from")); Serial.println(rf95.headerFrom());
							eeprom_write_block((void*)RSSItot, (void *)EE_ADDRESS_RSSI, sizeof(RSSItot));
							allNodes++;
						} else {
							LoraTo = asknode;
							byte c = mycode; mycode = LoraFrom;
							LoraSend((char*)buf);
							mycode = c;
						}
					}
			}
			//Serial.print('-');
			// scan all nodes up to nNodes
			if (scanmode) {                   //////start myscan
				int result[N_NODES] = { 0 };
				for (byte node = 0; node < nNodes; node++) {
					for (byte n = 0; n < nTimes; n++)
						if (node != mycode) {
							Serial.print(F("Scanning node ")); Serial.println(node);
							
							LoraTo = node;
							char mess[15] = "scan,", cc[8];
							strcat(mess, itoa(nNodes, cc, 10));
							Serial.println(mess);
							LoraSend(mess);
							// ---------------wait for node answers------------------------
							LoraFrom = node;
							unsigned long start = millis();
							bool done = false;
							result[node] += 150;
							while ((millis() < start + INTERVAL) && !done)
								if (readLora(buf, len)) {
									char* str = "          ";
									str = strtok((char*)buf, ",\0");
									if (str != NULL)
										if (strcmp(str, "RSSI") == 0) {
											str = strtok(NULL, ",\0");
											if (str != NULL) {
												byte r = atol(str);
												result[node] += r - 150;
												//	RSSItot[inde(node, mycode)] = atoi(str);
												Serial.print(RSSI[node]);
												Serial.print(F("RSSI from")); Serial.println(node);
												done = true;
											}
										}
								}
						}
					RSSI[node] = result[node] / nTimes;
					if (RSSI[node] > LIMIT_RSSI)RSSI[node] = 0;
					if( RSSI[node]!=0){
						oss.lcd.setCursor(0, 18);
						oss.lcd.printf("node %d RSSI %d t.%d", LoraFrom,RSSI[node], ++totn);
						oss.lcd.display();
					}
				}
				scanmode = false;
				LoraTo = asknode;
				Serial.println(asknode);
				//---------------------------send RSSI matrix to asknode---------------------------- or
				//---if mycode==0 store RRSI in general RSSItot matrix and sever on EEprom-----------------
				char  mess[50] = "RSSIres,", nn[10];
				strcat(mess, itoa(mycode, nn, 10)); strcat(mess, ",");
				for (byte i = 0; i < nNodes; i++) {
					strcat(mess, itoa(RSSI[i], nn, 10));
					strcat(mess, ",");
				}
				if (mycode > 0)LoraSend(mess);
				else {
					for (byte i = 1; i < nNodes; i++) {
						RSSItot[inde(i, 0)] = RSSI[i];
						Serial.print(inde(i, 0)); Serial.print(' '); Serial.println(RSSItot[inde(i, 0)]);
					}
					allNodes++;
					eeprom_write_block((void*)RSSItot, (void *)EE_ADDRESS_RSSI, sizeof(RSSItot));
					Serial.println(mess);
					//send message back with all RSSI
				}
			}

		}
		byte k = 0;
		for (byte i = 0; i < allNodes; i++)
		{	for (byte j = 0; j <= i; j++)
				Serial.printf("%d ", RSSItot[k++]);
		Serial.println();
		}
		//______________ end scanning load routings______________________________
		if(n_nodes==N_NODES)eeprom_write_byte(EE_ADDRESS_N_NODES,allNodes);
		eeprom_read_block((void*)RSSItot, (void *)EE_ADDRESS_RSSI, sizeof(RSSItot));
		maskRSSI(n_nodes,RSSItot);
		oss.lcd.setCursor(0, 36);
		for (byte i = 1; i < n_nodes; i++) {
			byte j = 0;

			for (j = 0; j < n_nodes; j++) {
				Serial.print(RSSItot[inde(i, j)]); Serial.print(' ');
				if (i == j)oss.lcd.print(" X ");
				else
				oss.lcd.printf("%2d ", RSSItot[inde(i, j)]);
				//if (RSSItot[inde(i, j)] != 0)break;
			}
			Serial.println();
			//oss.lcd.println();
			if (RSSItot[inde(i, 0)] == 0)
				//	if (j < n_nodes)
			{
			//	maskRSSI(n_nodes);
				Serial.print(F("path to")); Serial.println(i);
				
				byte nn = pathTo(0, i,3);
				routing[i] = 0;
				for (byte n = 2; n <= nn; n++) {
				Serial.println(node[n]);
				routing[i] += node[n]<<(n-2)*8 ;
				}
				Serial.println(routing[i], HEX);
				oss.lcd.println(routing[i], HEX);
			} else
				//Serial.print(F("node not found")); 
			{	Serial.print(' ');
			    oss.lcd.println(' ');
			}
		}
	}
}
#define SEND_COUNT 3
byte LoraStation::sendProgram(byte Address, char * buf) {
}

byte LoraStation::runValve(byte id, byte val, long time) {
	/// send //pr?pid=81&durs=time1,time2
	
	
	char mess[40], buff[100];
	unsigned long timer[2] = { 0, 0};
	timer[val]=time;
	byte count = 0;
	sprintf(mess, "/rp?pid=81&durs=%d,%d", timer[0],timer[1]);
	DEBUG_PRINTLN(mess);
	do {
		LoraSend(id, mess);
		count++;
	} while  (getAck(id, buff, sizeof(buff))==0&&count<SEND_COUNT);

	return 1-count/SEND_COUNT ; // return 0 if fail!! 
}
byte LoraStation::stopValves(byte id) {
	/// send //pr?pid=-1
	char mess[20] = "/rp?pid=-1", buff[100];
	byte count = 0;
	do {
		LoraSend(id, mess);
		count++;
	} while (getAck(id, buff, sizeof(buff))==0 && count<SEND_COUNT);
	return 1 - count / SEND_COUNT; // return 0 if fail!! LoraSend(id, mess);
}

byte LoraStation::runProgram(byte Address, byte pid) {
	return byte();
}

byte LoraStation::sendOptions(byte id, char * buf) {
	return byte();
}

byte LoraStation::getResponse(byte id, char * buf) {
	return byte();
}
#define WAIT_MILLI 2000
byte LoraStation::getAck(byte id, char * buf,byte len) {//read return message return message len or 0
	char* str="    ";
	byte  nn = 0;
	unsigned long millisout = millis() + WAIT_MILLI;
	while (millis() < millisout) {
		nn = readLora((byte*)buf, len);
		if (nn) {
			str = strtok(buf, "/");
			Serial.print(str); Serial.print(" from "); Serial.println(rf95.headerFrom());
		//	if (str == NULL)
				if(id != rf95.headerFrom())return 0;
		//	else
		//		if(id != atoi(str))return 0;
			break;
		}
	}
	return nn;
}

byte LoraStation::send(byte id, char * buf) {
	
	LoraSend(id,buf);
	return 1;
}

byte LoraStation::sendTime(byte id,unsigned long time) {
	/// send //cc?t=utc_time
	char mess[20], buff[100];
	sprintf(mess, "/cc?&dul", time);
	byte count = 0;
	do {
		LoraSend(id, mess);
		count++;
	} while (getAck(id, buff, sizeof(buff)) && count<SEND_COUNT);
	return 1 - count / SEND_COUNT; // return 0 if fail!! 
}

byte LoraStation::getVcontrol(byte id) {
	return byte();
}

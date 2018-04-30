#pragma once
#define N_STATIONS 5
#include <Arduino.h>
#include <RH_RF95.h>

static byte LoraTo = 1, LoraFrom = 0;
struct valve {
	byte number;
	String name;
};
class LoraStation {
public:
	LoraStation();
	~LoraStation();
	bool begin(byte cadmode);
	bool findRouting(byte n_nodes, bool  find_node_routing,byte all);
	byte StationAddress[N_STATIONS];           /// fixed at startup  by 
	String StationName[N_STATIONS];    
	valve  valv[4][N_STATIONS];
	long routing[N_STATIONS];                  // byte addr=(routing>>8)&0xFF ==0 direct
	byte sendProgram(byte id,char* buf);
	byte runValve(byte id, byte val,long time);
	byte stopValves(byte id);
	byte runProgram(byte id, byte pid);
	byte sendOptions(byte id, char *buf);
	byte getResponse(byte id, char*buf);
	byte send(byte id, char *buf);
	byte getAck(byte id, char *mess,byte len);
	byte getVcontrol(byte id);//list of values);
	byte sendTime(byte id,unsigned long time);
	void LoraSend(String s);
	void LoraSend(char * s);
	void LoraSend(byte id, char * buf);


	byte readLora(byte * buf, uint8_t len);

};


#pragma once


void ip2string(char * str, byte ip[4]);
void push_message(byte type, uint32_t lval = 0, float fval = 0.f, const char* sval = NULL);

void cloud_json_stations_attrib(const char * name, int addr, char * postval_pt);
byte push_message_cloud(byte type, ulong day = 0);
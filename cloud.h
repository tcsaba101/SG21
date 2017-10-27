#ifndef CLOUD_H

	#define CLOUD_H
	
	#include "defines.h"
	#include "OpenSprinkler.h"
	#include <SDFat.h>

	class cloud {
	public:
		//void ip2string(char * str, byte ip[4]);
		//void push_message(byte type, uint32_t lval = 0, float fval = 0.f, const char* sval = NULL);

		static void cloud_json_stations_attrib(const char * name, int addr, char * postval_pt);
		static byte push_message_cloud(byte type, ulong day = 0);
	};
#endif
#include <configPic.h>
#include <M95.h>
#include <modbus_485.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlibm.h>

unsigned int16 temp[4];

#int_rda2
void interruptReceiveModbus(){
	unsigned char c;
	//output_low(pinControl485);
	if(kbhit(MODBUS)){
		output_high(dataLed);
		c = fgetc(MODBUS);
		fputc(c, DEBUG);
		if(modbusSerialState == MODBUS_GETADDY){
			modbusSerialCrc.d = 0xffff;
			modbusRx.address = c;
			modbusRx.len = 0;
			modbusRx.error = 0;
			modbusSerialState = MODBUS_GETFUNC;
		}
		else if(modbusSerialState == MODBUS_GETFUNC){
			modbusRx.func = c;
			modbusRx.len = 0;
			modbusSerialState = MODBUS_GETDATA;
		}
		else if(modbusSerialState == MODBUS_GETDATA){
			modbusRx.data[modbusRx.len] = c;
			modbusRx.len++;
			if(modbusRx.len > 64)	modbusRx.len = 0;
		}
		modbusEnableTimeout(true);
		modbusCalcCrc(c);
		modbusSerialWait = MODBUS_SERIAL_TIMEOUT;
		if((modbusSerialCrc.b[0] == 0x00) && (modbusSerialCrc.b[1] == 0x00)){
			sendMessageComplete = true;
			//fputc(modbusserialcrc.b[0], MODBUS);
			//fputc(modbusSerialCrc.b[1], MODBUS);
			output_low(dataLed);
			modbusSerialState = MODBUS_GETADDY;

		}
		modbusResponseOK = false;
		modbusQuery = true;
		output_low(dataLed);
	}
}

#define modbusSerialWaitResponse() {\
	modbusTimeoutOK = false;\
	modbusTimeoutEnable = false;\
	output_low(pinControl485);\
	if(address){\
		while((!modbus_kbhit()) && (--modbusSerialWait)){\
			modbusResponseOK = false;\
		}\
		if(!modbusSerialWait){\
			modbusRx.error = TIMEOUT;\
			modbusTimeoutOK = false;\
			modbusSerialWait = MODBUS_SERIAL_TIMEOUT;\
		}\
	}\
}\

void powerOnM95(){
	fprintf(DEBUG, "Power On Module SIM M95\r\n");
	output_high(PKEY);
	output_low(PKEY);
	delay_ms(2000);
//	output_high(PKEY);
	output_high(pwrLed);
}

char serialEvent(){
	unsigned int16 iteraction;
	char inchar;
	if(kbhit(SIMM95)){
		output_high(dataLed);
		inchar = fgetc(SIMM95);
		if(tcpFlag == false){
			if(index < 200){
				inputString[index++] = inchar;
//				fputc(inchar, DEBUG);
			}
			if(inchar == '\n'){
				inputString[index] = 0;
				index = 0;
				fprintf(DEBUG, inputString);			
				if(strstr(inputString, (char *)("OK")) != 0){
					gsmResponse = 1;
				}
				else if(strstr(inputString, (char*)("ERROR")) != 0){
					gsmResponse = 2;
					tcpFlag = false;
				}
				else if(strstr(inputString, (char*)(".")) != 0){
					gsmResponse = 3;
				}
				else if(strstr(inputString, (char*)("CONNECT FAIL")) != 0){
					gsmResponse = 5;
				}
				else if(strstr(inputString, (char*)("CONNECT OK")) != 0){
					tcpFlag = true;
					pingFlag = true;
					gsmResponse = 4;
					fprintf(DEBUG, "TCP Connected\r\n");
					autoConnect();
					tcpATErrorCount = 0;
					output_high(linkLed);
					return;
				}
				else if(strstr(inputString, (char*)("CONNECT\r\n")) != 0){
					tcpFlag = true;
					pingFlag = true;
					gsmResponse = 4;
					fprintf(DEBUG, "TCP Connected\r\n");
					autoConnect();
					tcpATErrorCount = 0;
					output_high(linkLed);
					return;
				}
				else if(strstr(inputString, (char*)("CLOSED")) != 0){
					gsmResponse = 4;
					fprintf(DEBUG, "TCP Disconnected\r\n");
					tcpFlag = false;
					pingFlag = false;
					output_low(linkLed);
				}
				else if(strstr(inputString, (char*)(">")) != 0){
					gsmSendResponse = 1;
					fprintf(DEBUG, "content message:\r\n");
				}
				else if(strstr(inputString, reply) != 0){
					gsmResponseFlag = 1;
					if(strstr(inputString, (char *)(" INITIAL")) !=0){
						gsmResponseFlag = 2;
					}
					else if(strstr(inputString, (char*)(" START")) != 0){
						gsmResponseFlag = 3;
					}
					else if(strstr(inputString, (char*)(" CONFIG")) != 0){
						gsmResponseFlag = 4;
					}
					else if(strstr(inputString, (char*)(" GPRSACT")) != 0){
						gsmResponseFlag = 4;
					}
					else if((strstr(inputString, (char*)(" STATUS")) != 0) || (strstr(inputString, (char*)(" CLOSE")) != 0)){
						gsmResponseFlag = 5;
					}
					else if(strstr(inputString, (char*)("TCP CONNECTING")) != 0){
						gsmResponseFlag = 6;
					}
					else if((strstr(inputString, (char*)(" CONNECT OK")) != 0) || (strstr(inputString, (char*)(" CONNECT FAIL")) != 0) || (strstr(inputString, (char*)(" PDP DEACT")) != 0)){
						gsmResponseFlag = 7;
					}
				}
			}
		}
		else{
			unsigned int mqttControlPacketType = (inchar / 16) & 0x0f;
			packetType = mqttControlPacketType;
			unsigned int	DUP = (inchar & dupMask) / dupMask;
			unsigned int	QoS = (inchar & QoSMask) / QoSScale;
			unsigned int	Retain = inchar & retainMask;
			if((mqttControlPacketType >= CONNECT) && (mqttControlPacketType <= DISCONNECT)){
				bool nextLengthByte = true;
				len = 0;
				lengthLocal = 0;
				unsigned int32 multiplier = 1;
				int8 cchar = inchar;
				unsigned int32 temp = 0;
				while((nextLengthByte == true) && (tcpFlag == true)){
					if(kbhit(SIMM95)){
						output_high(dataLed);
						inchar = fgetc(SIMM95);
						if((((cchar & 0xff) == 'C') &&((inchar & 0xff) == 'L') && (len == 0)) || (((cchar & 0xff) == '+') && ((inchar & 0xff) && 'P') && (len == 0))){
							index = 0;
							inputString[index++] = cchar;
							inputString[index++] = inchar;
							tcpFlag = false;
							mqttFlag = false;
							pingFlag = false;
						}
						else{
							if((inchar & 128) == 128){
								len += (inchar & 127) * multiplier;
								multiplier *= 128;
							}
							else{
								nextLengthByte = false;
								len += (inchar & 127) * multiplier;
								multiplier *= 128;
							}
						}
					}
					temp++;
					if(temp > 10000)	break;
				}
			}
			lengthLocal = len;
			if(len > 200)	return;
			if(tcpFlag == true){
				index = 0;
//				memset(inputString, 0, sizeof(inputString));
				while(len > 0){
					inputString[index++] = fgetc(SIMM95);
					output_high(dataLed);
					len--;
					if(index > (sizeof(inputString) - 1))	break;
				}
				fprintf(DEBUG, "inputString: %d\r\n", inputString);
				switch(mqttControlPacketType){
					case CONNACK:{						
						fprintf(DEBUG, "reponse CONNACK: %d\r\n", mqttControlPacketType);
						fprintf(DEBUG, "inputString[0]: %d\r\n", inputString[0]);
						fprintf(DEBUG, "inputString[1]: %d\r\n", inputString[1]);
						connectionAcknowledgement = inputString[0] * 256 + inputString[1];
						if(connectionAcknowledgement == 0){
							mqttFlag = true;
						}
						connectReturnCode(connectionAcknowledgement);
						break;
					}
					case PUBLISH:{
						fprintf(DEBUG, "reponse PUBLISH: %d\r\n", mqttControlPacketType);
						fprintf(DEBUG, "inputString[0]: %d\r\n", inputString[0]);
						fprintf(DEBUG, "inputString[1]: %d\r\n", inputString[1]);
						unsigned int16 topicLength = inputString[0] * 256 + inputString[1];
						fprintf(DEBUG, "topicLength: %d\r\n", topicLength);
						if(topicLength > index){
							topicLength = 0;
							fprintf(DEBUG, "topic length is large\r\n");
							return;
						}
						unsigned int16 publishIndex = 0;
						memset(t_topic, 0, sizeof(t_topic));
						for(iteraction = 2; iteraction < topicLength; iteraction++){
							t_topic[publishIndex++] = inputString[iteraction];
						}
						iteraction = 0;
						fprintf(DEBUG, "t_tpoic1: %s\r\n", t_topic);
						t_topic[publishIndex] = 0;
						topicLength = publishIndex;
						publishIndex = 0;
						unsigned int16 messageStart = topicLength + 2;
						unsigned int16 messageID	 = 0;
						if(QoS == 0){
							messageStart = messageStart + 2;
							messageID = inputString[topicLength + 2] * 256 + inputString[topicLength + 3];
							fprintf(DEBUG, "messageID: %d\r\n", messageID);
						}
						memset(t_message, 0, sizeof(t_message));
						for(iteraction = messageStart; iteraction < lengthLocal; iteraction++){
							t_message[publishIndex++] = inputString[iteraction];
						}
						fprintf(DEBUG, "t_message2: %s\r\n", t_message);
						t_message[publishIndex] = 0;
						messageLength = publishIndex;
						publishIndex = 0;
						if(QoS == 1){
							publishACK(messageID);
							fprintf(DEBUG, "QoS: %d\r\n", QoS);
							fprintf(DEBUG, "messageID: %s\r\n", messageID);
						}
						if(QoS ==2){
							publishREC(messageID);
							fprintf(DEBUG, "QoS: %d\r\n", QoS);
							fprintf(DEBUG, "messageID: %s\r\n", messageID);
						}
						messageFlag  = true;
						break;
					}
					case PUBACK:{
						fprintf(DEBUG, "response PUBACK: %d\r\n", mqttControlPacketType);
						unsigned int16 packetID = inputString[2] * 256 + inputString[3];
						fprintf(DEBUG, "packet ID: %d\r\n", packetID);
						break;
					}
					case PUBCOMP:{						
						fprintf(DEBUG, "response PUCOMP: %d\r\n", mqttControlPacketType);
						unsigned int16 packetID = inputString[2] * 256 + inputString[3];
						fprintf(DEBUG, "packet ID: %d\r\n", packetID);
						break;
					}
					case SUBACK:{
						fprintf(DEBUG, "response SUBACK : %d\r\n", mqttControlPacketType);
						unsigned int16 packetID = inputString[2] * 256 + inputString[3];
						fprintf(DEBUG, "packet ID: %d\r\n", packetID);
						messageFlag = true;
						break;
					}
					case UNSUBACK:{										
						fprintf(DEBUG, "reponse UNSUBACK: %d\r\n", mqttControlPacketType);
						fprintf(DEBUG, "inputString[2]: %d\r\n", inputString[2]);
						fprintf(DEBUG, "inputString[3]: %d\r\n", inputString[3]);
						unsigned int16 packetID = inputString[2] * 256 + inputString[3];
						fprintf(DEBUG, "topicLength: %d\r\n", packetID);
						break;
					}
					case PUBREC:{						
						fprintf(DEBUG, "reponse PUBREC: %d\r\n", mqttControlPacketType);
						fprintf(DEBUG, "inputString[2]: %d\r\n", inputString[2]);
						fprintf(DEBUG, "inputString[3]: %d\r\n", inputString[3]);
						unsigned int16 messageID = inputString[2] * 256 + inputString[3];
						fprintf(DEBUG, "messageID: %d\r\n", messageID);
						publishREL(0, messageID);
						break;
					}
					case PUBREL:{										
						fprintf(DEBUG, "reponse PUBREL: %d\r\n", mqttControlPacketType);
						fprintf(DEBUG, "inputString[2]: %d\r\n", inputString[2]);
						fprintf(DEBUG, "inputString[3]: %d\r\n", inputString[3]);
						unsigned int16 messageID = inputString[2] * 256 + inputString[3];
						fprintf(DEBUG, "messageID: %d\r\n", messageID);
						publishCOMP(messageID);
						break;
					}
					case PINGREQ:{					
						fprintf(DEBUG, "reponse PINGREQUEST: %d\r\n", mqttControlPacketType);
						tcpFlag = false;
						pingFlag = false;
						fprintf(DEBUG, "Disconnecting...\r\n");
						modemStatus = 0;
						break;
					}
					case PINGRESP:{
						fprintf(DEBUG, "response PINGRESPONSE: %d\r\n", mqttControlPacketType);
						break;
					}
				}
			}
		}
		output_low(dataLed);
	}	
}

void connectReturnCode(unsigned int ack){
	fprintf(DEBUG, "Connect Return Code: 0x%2x\r\n", ack);
	switch(ack){
		case 0:{
			fprintf(DEBUG, "Connect Accepted.\r\n");
			break;
		}
		case 1:{
			fprintf(DEBUG, "Connection Refused, reason: Unacceptale Protocol Version\r\n");
			break;
		}
		case 2:{
			fprintf(DEBUG, "Connection Refused, reason: Identifier Rejected\r\n");
			break;
		}
		case 3:{
			fprintf(DEBUG, "Connection Refused, reason: Server Unavailable\r\n");
			break;
		}
		case 4:{
			fprintf(DEBUG, "Connection Refused, reason: Bad User Name of Password\r\n");
			break;
		}
		case 5:{
			fprintf(DEBUG, "Connection Refused, reason: Not Authorized");
			break;
		}
		default:{
			fprintf(DEBUG, "Reserved for future use\r\n");
			break;
		}
	}
}

char sendAT(char * command, unsigned long waitms){
	unsigned int16 currentTicks;
	gsmResponse = 0;
	fprintf(DEBUG, "Send Command:");
	fprintf(DEBUG, command);
	fprintf(SIMM95, command);
	set_ticks(T1, 0);
	currentTicks = get_ticks(T1);
	while((gsmResponse == 0) && (currentTicks < waitms)){
		serialEvent();
		currentTicks = get_ticks(T1);
	}
	return gsmResponse;
}

char sendATResponse(char *command, char *responseStr, unsigned long waitms){
	unsigned int16 currentTicks;
	memset(reply, 0, sizeof(reply));
	sprintf(reply, responseStr);
	gsmResponseFlag = 0;
	fprintf(DEBUG, "response function:");
	fprintf(DEBUG, command);
	fprintf(SIMM95, command);
	set_ticks(T1, 0);
	currentTicks = get_ticks(T1);
	while((gsmResponseFlag == 0) && (currentTicks < waitms)){
		serialEvent();
		currentTicks = get_ticks(T1);
	}
	return gsmResponseFlag;
}

void sendLength(unsigned long length){
	bool lengthFlag = false;
	while(lengthFlag == false){
		if((length / 128) > 0){
			fputc((length % 128 + 128), SIMM95);
			fprintf(DEBUG, "Send Length: %d", length);
			length /= 128;
		}
		else{
			fputc((length), SIMM95);
			fprintf(DEBUG, "Send Length: %d", length);
			lengthFlag = true;
		}
	}
}

void sendUTF8String(char *string){
	int lengthString = strlen(string);
	fputc((lengthString / 256), SIMM95);
	fputc((lengthString % 256), SIMM95);
	fprintf(SIMM95, string);
	
	fprintf(DEBUG, "Send Length String: %d\r\n", lengthString);
	fprintf(DEBUG, "SendUTF8String: %s\r\n", string);
}

void publishACK(unsigned int16 messageID){
	fprintf(DEBUG, "publishACK: ");
	fprintf(DEBUG, "messageID: %d\r\n", messageID);
	fputc((PUBACK * 16), SIMM95);
	sendLength(2);
	fputc((messageID / 256), SIMM95);
	fputc((messageID % 256), SIMM95);
}

void publishREC(unsigned int16 messageID){
	fprintf(DEBUG, "publishREC: ");
	fprintf(DEBUG, "messageID: %d\r\n", messageID);
	fputc((PUBREC * 16), SIMM95);
	sendLength(2);
	fputc((messageID / 256), SIMM95);
	fputc((messageID % 256), SIMM95);
}

void publishREL(unsigned int1 DUP, unsigned int16 messageID){
	fprintf(DEBUG, "publishREL: ");
	fprintf(DEBUG, "messageID: %d\r\n", messageID);
	fputc((PUBREL * 16 + DUP * dupMask + 1 * QoSScale), SIMM95);
	sendLength(2);
	fputc((messageID / 256), SIMM95);
	fputc((messageID % 256), SIMM95);
}

void publishCOMP(unsigned int16 messageID){
	fprintf(DEBUG, "publishCOMP: ");
	fprintf(DEBUG, "messageID: %d\r\n", messageID);
	fputc((PUBCOMP * 16), SIMM95);
	sendLength(2);
	fputc((messageID / 256), SIMM95);
	fputc((messageID % 256), SIMM95);
}

void pingRequest(){
	unsigned long currentTicks = get_ticks(T2);
	if(pingFlag){
		if((currentTicks - pingPrevTicks) >= keepAliveTimeout*1000/4){
			pingPrevTicks = currentTicks;	
			fprintf(DEBUG, "pingRequest: \r\n");
			fputc((PINGREQ * 16), SIMM95);
			sendLength(0);
		}
	}
}

void pingResponse(){	
	fprintf(DEBUG, "pingResponse: \r\n");
	fputc((PINGRESP * 16), SIMM95);
	sendLength(0);
}

void mqttDisconnect(){
	fprintf(DEBUG, "MQTT Disconnect\r\n");
	fputc((DISCONNECT * 16), SIMM95);
	sendLength(0);
	pingFlag = false;
}

void mqttSubscribe(unsigned int1 DUP, unsigned int16 messageID, char *subtopic, unsigned int8 subQoS){
	fprintf(DEBUG, "MQTT SUBSCRIBE: ");
	fputc((SUBSCRIBE * 16 + DUP * dupMask + 1 * QoSScale), SIMM95);
	int localLength = 2 + (2 + strlen(subtopic)) + 1;
	sendLength(localLength);
	fputc((messageID / 256), SIMM95);
	fputc((messageID % 256), SIMM95);
	fprintf(DEBUG, "messageID: %d\r\n", messageID);
	sendUTF8String(subtopic);
	fputc(subQoS, SIMM95);
}

void mqttUnsubscribe(unsigned int1 DUP, unsigned int16 messageID, char *topic){
	fprintf(DEBUG, "MQTT UNSUBSCRIBE: ");
	fputc((UNSUBSCRIBE * 16 + DUP * dupMask + 1*QoSScale), SIMM95);
	int localLength = 2 + 2 + strlen(localLength);
	sendLength(localLength);
	fputc((messageID / 256), SIMM95);
	fputc((messageID % 256), SIMM95);
	fprintf(DEBUG, "messageID: %d\r\n", messageID);
	sendUTF8String(topic);
}

void mqttPublish(unsigned int1 DUP, unsigned int8 QoS, unsigned int1 retain, unsigned int16 messageID, char *topic, char *message){
	fprintf(DEBUG, "MQTT PUBLISH: ");
	fputc((PUBLISH * 16 + DUP * dupMask + QoS * QoSScale + retain), SIMM95);
	int localLength = 2 + strlen(topic);
	if(QoS > 0){
		localLength += 2;
	}
	localLength += strlen(message);
	sendLength(localLength);
	sendUTF8String(topic);
	if(QoS > 0){
		fputc(messageID / 256, SIMM95);
		fputc(messageID % 256, SIMM95);
		fprintf(DEBUG, "messageID: %d\r\n", messageID);
	}
	fprintf(SIMM95, message);
	fprintf(DEBUG, "PubTopic: %s\r\n", topic);
	fprintf(DEBUG, "Message Publish: %s\r\n"message);
}

unsigned int16 generateMessageID(){
	if(lassMessageID < 65535){
		return ++lassMessageID;
	}
	else{
		lassMessageID = 0;
		return lassMessageID;
	}
}

void mqttConnect(client	*mqttClient){
	connectionAcknowledgement = NOACKNOWLEDGEMENT;
	fprintf(DEBUG, "Connect send\r\n");
	fprintf(DEBUG, "Client ID: %s\r\n", mqttClient -> mqttClientID);
	fprintf(DEBUG, "Will Flag: 0x%2x\r\n", mqttClient -> willFlag);
	fprintf(DEBUG, "UserName Flag: 0x%2x\r\n", mqttClient -> userNameFlag);
	fprintf(DEBUG, "KeepAliveTimeout: %d\r\n", keepAliveTimeout);
	
	fputc((CONNECT * 16), SIMM95);
	char protocolName[7] = "MQTT";
	int localLength = (2 + strlen(protocolName)) + 1 + 3 + (2 + strlen(mqttClient -> mqttClientID));
	if(mqttClient -> willFlag != 0){
		localLength = localLength + (2 + strlen(mqttClient -> willTopic) 
												+ (2 + strlen(mqttClient -> willMessage)));
	}
	if(mqttClient -> userNameFlag != 0){
		localLength = localLength + (2+ strlen(mqttClient -> userName));
		if(mqttClient -> passwordFlag != 0){
			localLength = localLength + (2 + strlen(mqttClient -> password));
		}
	}
	
	sendLength(localLength);
	sendUTF8String(protocolName);
	fputc((protocolVersion), SIMM95);
	fputc(((mqttClient -> userNameFlag * userNameFlagMask) 
						+ (mqttClient -> passwordFlag * passwordFlagMask) 
						+ (mqttClient -> willRetain * willRetainMask) 
						+ (mqttClient -> willQoS * willQoSScale) 
						+ (mqttClient -> willFlag * willFlagMask) 
						+ (mqttClient -> cleanSession * cleanSessionMask)), SIMM95);
	fputc((keepAliveTimeout / 256), SIMM95);
	fputc((keepAliveTimeout % 256), SIMM95);
	sendUTF8String(mqttClient -> mqttClientID);
	if(mqttClient -> willFlag == 1){
		sendUTF8String(mqttClient -> willTopic);
		sendUTF8String(mqttClient -> willMessage);
	}
	if(mqttClient -> userNameFlag == 1){
		sendUTF8String(mqttClient -> userName);
		if(mqttClient -> passwordFlag == 1){
			sendUTF8String(mqttClient -> password);
		}
	}
}

void autoConnect(){
	mqttConnect(mqttClient);
}

void tcpInit(){
	unsigned int16 currentTicks = 0;
	char *commandAT;
	char *strrep;
	commandAT = malloc(200);
	strrep = malloc(20);
	memset(commandAT, 0, sizeof(commandAT));
	memset(strrep, 0, sizeof(strrep));
	switch(modemStatus){
		case 0:{
			sendAT((char*)("+++"), 2000);
			fprintf(DEBUG, "Sending AT\r\n");
			memset(commandAT, 0, sizeof(commandAT));
			sprintf(commandAT, "AT\r\n");
			if(sendAT(commandAT, 5000) == 1){
				modemStatus = 1;
			}
			else{
				modemStatus = 0;
				break;
			}
		}
		case 1:{
			sprintf(commandAT, "ATE0\r\n");
			if(sendAT(commandAT, 2000) == 1){
				modemStatus = 2;
			}
			else{
			modemStatus = 1;
			break;
			}
		}
		case 2:{
			memset(commandAT, 0, sizeof(commandAT));
			memset(strrep, 0, sizeof(strrep));
			sprintf(commandAT, "AT+CREG?\r\n");
			sprintf(strrep, "+CREG: 0,1");
			if(sendATResponse(commandAT, strrep, 8000) == 1){
				sendAT((char*)("AT+QIFGCNT=0\r\n"), 1000);
				sendAT((char*)("AT+QIDNSIP=0\r\n"), 1000);
				sendAT((char*)("AT+QINDI=1\r\n"), 1000);
				sendAT((char*)("AT+QIMUX=0\r\n"), 1000);
				sendAT((char*)("AT+QIMODE=1\r\n"), 1000);
				memset(commandAT, 0, sizeof(commandAT));
				memset(strrep, 0, sizeof(strrep));
				sprintf(commandAT, "AT+CGATT?\r\n");
				sprintf(strrep, "+CGATT: 1");
				if(sendATResponse(commandAT, strrep, 4000) != 1){
					sendAT((char*)("AT+CGATT=1\r\n"), 2000);
				}
				modemStatus = 3;
				tcpStatus = 2;
			}
			else{
				modemStatus = 2;
				break;
			}
		}
		case 3:{
			if(gsmResponseFlag != 7){
				memset(commandAT, 0, sizeof(commandAT));
				memset(strrep, 0, sizeof(strrep));
				sprintf(commandAT, "AT+QISTAT\r\n");
				sprintf(strrep, "STATE:");
				tcpStatus = sendATResponse(commandAT, strrep, 4000);
				if(tcpStatusPrev == tcpStatus){
					tcpATErrorCount++;
					if(tcpATErrorCount >= 10){
						tcpATErrorCount = 0;
						tcpStatus = 7;
					}
				}
				else{
					tcpStatusPrev = tcpStatus;
					tcpATErrorCount = 0;
				}
			}
			tcpStatusPrev = tcpStatus;
			fprintf(DEBUG, "%d\r\n", tcpStatus);
			switch(tcpStatus){
				case 2:{
					sendAT((char*)("AT+QIREGAPP=\"CMNET\"\r\n"), 5000);
					break;
				}
				case 3:{
					sendAT((char*)("AT+QIACT\r\n"), 5000);
					break;
				}
				case 4:{
					memset(commandAT, 0, sizeof(commandAT));
					memset(strrep, 0, sizeof(strrep));
					sprintf(commandAT, "AT+QILOCIP\r\n");
					sprintf(strrep, ".");
					sendATResponse(commandAT, strrep, 5000);
					break;
				}
				case 5:{
//					sendAT((char*)("AT + QIDNSIP = 1\r\n"), 2000);
					memset(mqttHost, 0, sizeof(mqttHost));
					memset(mqttPort, 0, sizeof(mqttPort));
					sprintf(mqttHost, "103.249.100.48");
					sprintf(mqttPort, "1883");
					memset(commandAT, 0, sizeof(commandAT));
					sprintf(commandAT, "AT+QIOPEN=\"TCP\",\"%s\",\"%s\"\r\n", mqttHost, mqttPort);
					if(sendAT(commandAT, 2000) == 1){
						memset(reply, 0, sizeof(reply));
						strcpy(reply, "none");
						set_ticks(T1, 0);
						currentTicks = get_ticks(T1);
						while((gsmResponse != 4) && (currentTicks < 10000)){
							serialEvent();
							currentTicks = get_ticks(T1);
						}
					}
					break;
				}
				case 6:{
					fprintf(DEBUG, "Waiting reconnect...\r\n");
					set_ticks(T1, 0);
					currentTicks = get_ticks(T1);
					while((gsmResponse != 4) && (currentTicks < 10000)){
						serialEvent();
						currentTicks = get_ticks(T1);
					}
					break;
				}
				case 7:{
					memset(commandAT, 0 sizeof(commandAT));
					memset(strrep, 0, sizeof(strrep));
					sprintf(commandAT, "AT+QIDEACT\r\n");
					sprintf(strrep, "OK");
					sendATResponse(commandAT, strrep, 4000);
					modemStatus = 0;
					tcpStatus = 2;
					break;
				}
			}
		}
	}
}

void begin(){
//	sendAT((char*)("AT+QIDEACT\r\n"), 2000);
//	delay_ms(4000);
	tcpInit();
}

void mqttProcessing(){
	if(tcpFlag == false){
		tcpFlag = false;
		tcpInit();
		return;
	}
	else{
		serialEvent();
	}
	pingRequest();
}

void messageProcessing(){
	if(messageFlag){
		char *c_topic, *c_message, *c_msg;
		c_topic = malloc(200);
		c_msg = malloc(200);
		memset(c_topic, 0, sizeof(c_topic));
		memset(c_msg, 0, sizeof(c_msg));
		if(t_topic > 200 || t_message > 200){
			messageFlag = false;
			return;
		}
		else{
			sprintf(c_topic, "%s", mqttClient -> mqttClientID);
			if(!strcmp(c_topic, t_topic)){
				fprintf(DEBUG, "received topic: %s\r\n", t_topic);
				if(!strcmp((char*)("ON"), t_message)){
					fprintf(DEBUG, "Lamp Status1: %s\r\n", t_message);
				}
				else if(!strcmp((char*)("OFF"), t_message)){
					fprintf(DEBUG, "Lamp Status2: %s\r\n", t_message);
				}
				messageFlag = false;
				return;
			}
			memset(c_topic, 0, sizeof(c_topic));
		}
	}
	messageFlag = false;
}

bool mqttAvailable(){
	return mqttFlag;
}

void modbusEnableTimeout(int1 enable){
	modbusTimeoutEnabled = enable;
	set_ticks(T3, 0);
}

void modbusTimeoutNow(){
	if((modbusSerialState == MODBUS_GETDATA) && (modbusSerialCrc.d == 0x0000) && (!modbusSerialNew)){
		modbusRx.len -= 2;
		modbusSerialNew = true;
//		modbusSerialState = MODBUS_GETADDY;
	}
	else{
		modbusSerialNew = false;
	}
	modbusSerialCrc.d = 0xffff;
	modbusSerialState = MODBUS_GETADDY;
	modbusEnableTimeout(false);
}

void modbusCheckTimeout(){
	get_ticks(T3);
	if(!modbusTimeoutEnabled && (get_ticks(T3) > MODBUS_GETDATA_TIMEOUT)){
		modbusTimeoutNow();
	}
}

bool modbus_kbhit(){
	modbusCheckTimeout();
	if (kbhit(MODBUS)) {
		modbusTimeoutOK = false;
		if (modbusRx.func == 0x80) {
			modbusRx.error = modbusRx.data[0];
			modbusRx.len = 1;
		}
		return true;
	}
	else {
		return false;
	}
	/*if(!modbusSerialNew){
		return false;
	}*/
	/*modbusSerialNew = false;
	return true;*/
}

void modbusCalcCrc(char data){
	unsigned int8 mIndex;
	mIndex = modbusSerialCrc.b[1] ^ data;
	modbusSerialCrc.b[1] = modbusSerialCrc.b[0] ^ modbus_auchCRCHi[mIndex];
	modbusSerialCrc.b[0] = modbus_auchCRCLo[mIndex];
}

exception modbusReadHoldingRegRsp(unsigned int8 address, unsigned int8 func, unsigned int16 startAddress, unsigned int16 quality){
	unsigned int8 startAddressHi, startAddressLo;
	unsigned int8 qualityHi, qualityLo;
	startAddressHi = make8(startAddress, 1);
	startAddressLo = make8(startAddress, 0);
	qualityHi = make8(quality, 1);
	qualityLo = make8(quality, 0);
	switch(caseSend){
		case 0:{
			disable_interrupts(int_rda2);
			output_high(pinControl485);
			modbusSerialCrc.d = 0xffff;
			modbusSerialNew = false;
			caseSend = 1;
			break;
		}
		case 1:{
			fputc(address, MODBUS);
			modbusCalcCrc(address);
			caseSend = 2;
			break;
		}
		case 2:{
			fputc(func, MODBUS);
			modbusCalcCrc(func);
			caseSend = 3;
			break;
		}
		case 3:{
			fputc(startAddressHi, MODBUS);
			modbusCalcCrc(startAddressHi);
			caseSend = 4;
			break;
		}
		case 4:{
			fputc(startAddressLo, MODBUS);
			modbusCalcCrc(startAddressLo);
			caseSend = 5;
			break;
		}
		case 5:{
			fputc(qualityHi, MODBUS);
			modbusCalcCrc(qualityHi);
			caseSend = 6;
			break;
		}
		case 6:{
			fputc(qualityLo, MODBUS);
			modbusCalcCrc(qualityLo);
			caseSend = 7;
			break;
		}
		case 7:{
			crcHigh = modbusSerialCrc.b[1];
			crcLow = modbusSerialCrc.b[0];
			fputc(crcHigh, MODBUS);
			modbusCalcCrc(crcHigh);
			caseSend = 8;
			break;
		}
		case 8:{
			crcLow = modbusSerialCrc.b[1];
			fputc(crcLow, MODBUS);
			modbusCalcCrc(crcLow);
			caseSend = 9;
			break;
		}
		case 9:{
			modbusSerialCrc.d = 0xffff;
			output_low(pinControl485);
			modbusSerialNew = true;
			modbusSerialWait = MODBUS_SERIAL_TIMEOUT;
			clear_interrupt(int_rda2);
			enable_interrupts(global);
			enable_interrupts(int_rda2);
			modbusSerialWaitResponse();
			sendMessageComplete = false;
			caseSend = 0;
			stack = 0;
			return modbusRx.error;
			break;
		}
	}
}

void main(){
	caseSend = 0;
	
	clear_interrupt(int_rda2);
	enable_interrupts(global);
	enable_interrupts(int_rda2);

	set_tris_f(0x42);
	output_low(pinControl485);
	set_tris_e(0x10);
	output_high(PKEY);
	
	memset(inputString, 0, sizeof(inputString));
	
	reply = malloc(20);
	memset(reply, 0, sizeof(reply));
	
	mqttClient = malloc(1024);
	memset(mqttClient, 0, sizeof(mqttClient));
	sprintf(mqttClient -> mqttClientID, "DeviceNo.1");
	mqttClient -> passwordFlag = false;
	mqttClient -> userNameFlag = false;
	sprintf(mqttClient -> password, "");
	sprintf(mqttClient -> userName, "");
	mqttClient -> cleanSession = 1;
	mqttClient -> willFlag = 0;
	mqttClient -> willQoS = 0;
	mqttClient -> willRetain = 0;
	sprintf(mqttClient -> willTopic, "");
	sprintf(mqttClient -> willMessage, "");
	keepAliveTimeout = 60;
	
	subTopicFlag = true;
	pubTopicFlag = true;
	messageFlag	= false;
	mqttQuery = true;
	
	mqttHost = malloc(50);
	mqttPort = malloc(10);
	memset(mqttHost, 0, sizeof(mqttHost));
	memset(mqttPort, 0, sizeof(mqttPort));
	
	fprintf(DEBUG, "Setting Broker\r\n");
	sprintf(mqttHost, "103.249.100.48");
	fprintf(DEBUG, "Broker: %s\r\n", mqttHost);
	sprintf(mqttPort, "1883");
	fprintf(DEBUG, "port: %s\r\n", mqttPort);
	fprintf(DEBUG, "Setting Broker OK\r\n");
	
	powerOnM95();
	
	pubTopic = malloc(100);
	subTopic = malloc(100);
	message = malloc(255);	
	memset(subTopic, 0, sizeof(subTopic));
	memset(pubTopic, 0, sizeof(pubTopic));
	memset(message, 0, sizeof(message));
	sprintf(subTopic, "%s", mqttClient -> mqttClientID);

	stack = 0;
	modbusQuery = true;
	modbusTimeoutOK = false;
	
	delay_ms(5000);
	tcpFlag = false;
	begin();
	while(1){
		messageProcessing();
		switch(stack){
			case 0:{
				stack = 1;
				if(tcpFlag){
//					serialEvent();
					if(mqttAvailable()){
						serialEvent();
						if(subTopicFlag && mqttQuery){
							memset(subTopic, 0, sizeof(subTopic));
							sprintf(subTopic, "%s", mqttClient -> mqttClientID);
							mqttSubscribe(0, generateMessageID(), subTopic, 0);
							subTopicFlag = false;
							mqttQuery = false;
						}
						if(pubTopicFlag && mqttQuery){
							fprintf(DEBUG, "mqtt Control Packet Type: %d\r\n", packetType);
							memset(pubTopic, 0, mqttQuery);
							//sprintf(pubTopic, "%s/temperature/value", mqttClient -> mqttClientID);
							sprintf(pubTopic, "/devices/temperature/value");
							/*if(numOfReg == 8){
								temp[RTD_CHANNEL_1] = modbusRx.data[1] * 256 + modbusRx.data[2];
								temp[RTD_CHANNEL_2] = modbusRx.data[3] * 256 + modbusRx.data[4];
								temp[RTD_CHANNEL_3] = modbusRX.data[5] * 256 + modbusRx.data[6];
								temp[RTD_CHANNEL_4] = modbusRx.data[7] * 256 + modbusRx.data[8];
							}*/
							//memset(message, 0, sizeof(message));
							//sprintf(message, "Current Temperature Value: %d.%d degreeC", dataHigh[RTD_CHANNEL_4], dataLow[RTD_CHANNEL_4]);
							//mqttPublish(0, 0, 0, generateMessageID(), pubTopic, message);
							sprintf(json, "{\"Temp\":\"%d.%d\"}", (temp[RTD_CHANNEL_4] / 10), (temp[RTD_CHANNEL_4] % 10));
							mqttPublish(0, 0, 0, generateMessageID(), pubTopic, json);
							pubTopicFlag = false;
							mqttQuery = false;
						}
					}
					else{
						subTopicFlag = true;
						pubTopicFlag = true;
						mqttQuery = true;
					}
				}
				break;
			}
			case 1:{
				output_high(pinControl485);
				if (modbusReadHoldingRegRsp(slaveID, FUNC_READ_HOLDING_REGISTERS, 40020, 4)) {
					numOfReg = modbusRx.data[0];
					fprintf(DEBUG, "\r\nnumOfReg: %d\r\n", numOfReg);
					if (numOfReg == 8) {
						temp[RTD_CHANNEL_1] = modbusRx.data[1] * 256 + modbusRx.data[2];
						temp[RTD_CHANNEL_2] = modbusRx.data[3] * 256 + modbusRx.data[4];
						temp[RTD_CHANNEL_3] = modbusRX.data[5] * 256 + modbusRx.data[6];
						temp[RTD_CHANNEL_4] = modbusRx.data[7] * 256 + modbusRx.data[8];
					}
				}
				else {
					fprintf(debug, "*** Exception: %d ***\r\n", modbusRx.error);
				}
				modbusquery = false;
				break;
			}
		}
		mqttProcessing();
		mqttQuery = true;
//		subTopicFlag = true;
		pubTopicFlag = true;
	}
	return;
}
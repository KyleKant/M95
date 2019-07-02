#ifndef __M95_H_
#define __M95_H_

#include <stdbool.h>

#define	CONNECT				1
#define	CONNACK				2
#define	PUBLISH				3
#define	PUBACK				4
#define	PUBREC				5
#define	PUBREL				6
#define	PUBCOMP				7
#define	SUBSCRIBE			8
#define	SUBACK				9
#define	UNSUBSCRIBE		10
#define	UNSUBACK			11
#define	PINGREQ				12
#define	PINGRESP			13
#define	DISCONNECT		14
#define	AUTH					15

#define	dupMask				8
#define	QoSMask				6
#define QoSScale				2
#define	retainMask			1

#define	userNameFlagMask	128
#define	passwordFlagMask		64
#define willRetainMask			32
#define	willQoSMask				24
#define	willQoSScale				8
#define	willFlagMask				4
#define	cleanSessionMask		2

#define	DISCONNECTED	0
#define	CONNECTED			1
#define	NOACKNOWLEDGEMENT	255

struct UART3struct{
	char rxDataString[256];
	unsigned int8	rxPointer;
}uart3;

typedef struct{
	char mqttClientID[100];
	bool	passwordFlag;
	bool	userNameFlag;
	char	password[64];
	char	userName[64];
	bool	cleanSession;
	bool	willFlag;
	bool	willQoS;
	bool	willRetain;
	char	willTopic;
	char	willMessage;
}client;

client		*mqttClient;

char inputString[256];
unsigned int index;
bool stringComplete = false;
bool	tcpFlag = false;
bool	reconnectFlag = false;

bool	mqttQuery = true;

bool	subTopicFlag;
bool	pubTopicFlag;
bool	messageFlag;
char *subTopic;
char *pubTopic;
char *message;

static char json[70];

char	*mqttHost;
char *mqttPort;

char *at;
char *strresponse;

unsigned int16	connectionAcknowledgement = NOACKNOWLEDGEMENT;

bool finishString = false;
bool mqttFlag = false;
bool pingFlag = false;
unsigned long	len;
unsigned long	lengthLocal;
unsigned long	messageLength = 0;
volatile unsigned int	protocolVersion = 4;	//MQTT Protocol version 3.1.1
volatile unsigned int	gsmResponse = 0;
volatile unsigned int	gsmResponseFlag = 0;
volatile unsigned int	modemStatus = 0;
volatile unsigned int	tcpStatus = 0;
volatile unsigned int 	tcpStatusPrev;
volatile unsigned int	gsmSendResponse = 0;
volatile unsigned int16 lassMessageID = 0;
unsigned int	tcpATErrorCount = 0;
unsigned long	keepAliveTimeout;
unsigned long	pingPrevTicks = 0;
unsigned int packetType;

unsigned int32 delay1 = 0, delay2 = 0;

char *reply;
char t_topic[100];
char t_message[256];
void powerOnM95();
char serialEvent();
char sendAT(char * command, unsigned long waitms);
char sendATResponse(char *command, char *responseStr, unsigned long waitms);
void tcpInit();
void mqttProcessing();
void begin();
void autoConnect();
void mqttConect(client *mqttClient);
bool mqttAvailable();
void publishACK(unsigned int16 messageID);
void publishREC(unsigned int16 messageID);
void publishREL(unsigned int1 DUP, unsigned int16 messageID);
void publishCOMP(unsigned int16 messageID);
void pingRequest();
void pingResponse();
void connectReturnCode(unsigned int ack);

#endif
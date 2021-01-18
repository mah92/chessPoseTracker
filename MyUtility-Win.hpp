/*
 * MyUtility functions for Windows
 * ver 0.99
 */

//boolean type/////////////////////////////////////////////
#ifndef __cplusplus
#define false 0
#define true 1
typedef int bool;
#endif

//Random number generation/////////////////////////////////
#include <stdlib.h>
#define frand() ((double) rand() / (RAND_MAX+1.0))

//Standard functions for logging///////////////////////////
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

FILE *logfile;
char messages_savepath[128];

bool Save2LogCat = true;
bool Save2File = true;
bool Save2TV = true;

void LOGOPEN(const char* savepath)
{
	if(Save2File){
		strcpy(messages_savepath, savepath);
		strcat(messages_savepath, "/messagelog.txt");
		logfile=fopen(messages_savepath, "a+");	//"a+":=append,read&write
		if(logfile==NULL)
			printf("LOGOPEN: Message Logger Could not Open .txt file\n");
	}
}

void LOGDUMP()
{
	if(Save2File)
		fflush(logfile);//fflush needed before closing for the buffer to be written to file
}

void LOGCLOSE()
{
	if(Save2File)
		fclose(logfile);
}

void LOGI(const char* mTAG, const char *format, ...)
{
	char logstr[400]="";
    va_list arg_list;

    va_start(arg_list, format);
	vsprintf(logstr, format, arg_list);
    va_end(arg_list);

    if(Save2LogCat){
        printf("%s", mTAG);
	printf("%s", logstr);
    }

	if(Save2File){
    	fprintf(logfile, "%s", mTAG);
	fprintf(logfile, "%s", logstr);
	}

}

void LOGW(const char* mTAG, const char *format, ...)
{
	char logstr[400]="";
    va_list arg_list;

    va_start(arg_list, format);
	vsprintf(logstr, format, arg_list);
    va_end(arg_list);

    if(Save2LogCat){
        printf("%sW:", mTAG);
	printf("%s", logstr);
    }
    if(Save2File){
        fprintf(logfile, "%sW:", mTAG);
	fprintf(logfile, "%s", logstr);
    }
}

void LOGE(const char* mTAG, const char *format, ...)
{
	char logstr[400]="";
    va_list arg_list;

    va_start(arg_list, format);
	vsprintf(logstr, format, arg_list);
    va_end(arg_list);

    if(Save2LogCat){
        printf("%sE:", mTAG);
	printf("%s", logstr);
    }
    if(Save2File){
        fprintf(logfile, "%sE:", mTAG);
	fprintf(logfile, "%s", logstr);
    	LOGDUMP();//Emergency Dump
    }
}

#ifdef __cplusplus
}
#endif

//LOCKS////////////////////////////////////////////////////
#define _LOCKC(Key,err)\
    if ((*env)->MonitorEnter(env, Key) != JNI_OK) {\
    	LOGW(TAG, " MonitorEnter != JNI_OK\n");\
    	return err;}

#define _UNLOCKC(Key,err)\
    if ((*env)->MonitorExit(env, Key) != JNI_OK) {\
    	LOGW(TAG, " MonitorExit != JNI_OK\n");\
    	return err;}

#define _LOCKCPP(Key,err)\
    if ((env)->MonitorEnter(Key) != JNI_OK) {\
    	LOGW(TAG, " MonitorEnter != JNI_OK\n");\
    	return err;}

#define _UNLOCKCPP(Key,err)\
    if ((env)->MonitorExit(Key) != JNI_OK) {\
    	LOGW(TAG, " MonitorExit != JNI_OK\n");\
    	return err;}

//Time/////////////////////////////////////////////////////
#include <time.h>
#include <math.h>

#define CLOCK_REALTIME 0
#define CLOCK_MONOTONIC 1
#define CLOCK_PROCESS_CPUTIME_ID 2
#define CLOCK_THREAD_CPUTIME_ID 3

#ifdef __cplusplus
extern "C" {
#endif

/*double _biasTime = 0;
void mysetTime(double currentTime)
{
	_biasTime = (currentTime) - ((double)clock()/CLOCKS_PER_SEC);
}
double myTime()
{
    return ((double)clock()/CLOCKS_PER_SEC) + _biasTime;
}*/
long _biasTime = 0;//in clock cycles
void mysetTime(double currentTime)
{
	_biasTime = (currentTime * CLOCKS_PER_SEC - (long)clock()) / (double)CLOCKS_PER_SEC;
}
double myTime()
{
    return (((long)clock() + _biasTime) / (double)CLOCKS_PER_SEC);
}

#ifdef __cplusplus
}
#endif

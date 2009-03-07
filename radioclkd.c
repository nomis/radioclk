/* radioclkd.c -- decode DCF77/MSF/WWVB time signal from a receiver attached to
 *                the serial port, and send to ntpd via shared memory reference
 *                clock driver.
 *
 * Copyright (c) 2001-03  Jonathan A. Buzzard (jonathan@buzzard.org.uk)
 *
 * The SHM code to interface with the NTP SHM reference clock driver is
 * derived from code by David J. Schwartz <davids@webmaster.com>
 *
 * The idea to take the average time of the best pulses in the last
 * minute is that of Jon Atkins <jon@jonatkins.com>
 *
 * The algorithm for UTCtime was taken from the mktime routine in libntp, though
 * none of the original code was used, with much of the detail removed for our
 * somewhat limited requirements here.
 *
 * Note: The DCF77 transmitter is located at 50:01N,9:00E
 *       The MSF transmitter is located at 52:22N,1:11W
 *       The WWVB transmitter is located at 40:40N,105:03W
 *       The HGB transmitter is located at 46:24N,6:15E
 *       The TDF transmitter is located at 47:10N,2:12E
 *       The JJY40 transmitter is located at 37:22N,140:51E
 *       The JJY60 transmitter is located at 33:28N,130:11E
 *
 * WARNING: This software is only year 2038 compliant if time_t and associated
 *          time functions on your operating system is 2038 compliant. Also
 *          note that this software is not year 3000 compliant. This is all
 *          presuming the transmitters are still operational that far in the
 *          future.
 *
 * $Log: radioclkd.c,v $
 * Revision 2.5  2003/01/20 16:48:33  jab
 * stricter testing of received WWVB code for errors
 * reject received times that differ by 1000 secs from system time
 *
 * Revision 2.4  2003/01/19 10:41:02  jab
 * fixed WWVB decoding so it works against a real signal
 * set timezone to UTC in testing mode so timestamps are actually UTC
 * changed CalculatePPSAverage to use 59 samples and fixed indexing bug
 * made sure clockInfo struct is properly reset at all appropriate points
 *
 * Revision 2.3  2002/11/16 21:03:35  jab
 * use own replacement for mktime(), so logs are in the local time
 *
 * Revision 2.2  2002/10/24 11:39:09  jab
 * fixed bugs in decoding of cts and dsr signals
 *
 * Revision 2.1  2002/10/17 22:34:28  jab
 * added pulse time averaging for improved performance
 * now decodes third clock on DSR as well
 *
 * Revision 2.0  2002/04/13 17:47:24  jab
 * inital dual receiver decoding version
 *
 * Revision 1.10  2002/04/10 15:36:43  jab
 * fixed array indexing bug in DecodeMSF
 *
 * Revision 1.9  2002/03/31 16:13:07  jab
 * fixed DCF77 and MSF decoders to deal with daylight savings
 *
 * Revision 1.8  2002/03/19 11:20:36  jab
 * make sure only one copy of radioclkd is running at any one time
 *
 * Revision 1.7  2002/03/12 21:56:04  jab
 * really fixed DCF77 decoding this time
 * backed out putting timestamps with LEAP_NOTINSYNC
 *
 * Revision 1.6  2002/03/05 09:48:54  jab
 * put timestamp with LEAP_NOTINSYNC on error to reduce log noise
 *
 * Revision 1.5  2002/03/04 12:58:15  jab
 * fixed decoding of the DCF77 signal so it actually works
 *
 * Revision 1.4  2002/02/08 15:21:11  jab
 * switched to using time_t internally to pass the time around
 *
 * Revision 1.3  2002/02/04 16:25:19  jab
 * added autodetection of which time code is being received
 * converted WWVB decoding to more compact method
 *
 * Revision 1.2  2002/02/01 12:21:10  jab
 * new more compact decoding routines for DCF77 and MSF
 * some bug fixes in PutTimeStamp
 *
 * Revision 1.1  2002/01/29 00:51:02  jab
 * Initial revision
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

static const char rcsid[]="$Id: radioclkd.c,v 2.5 2003/01/20 16:48:33 jab Exp jab $";

#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include<unistd.h>
#include<time.h>
#include<sys/time.h>
#include<termio.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<sys/mman.h>
#include<sys/ipc.h>
#include<sys/shm.h>
#include<fcntl.h>
#include<signal.h>
#include<sched.h>
#include<syslog.h>
#include<paths.h>
#include<string.h>
#include<setjmp.h>


#define PID_FILE _PATH_VARRUN "radioclkd.pid"

/*
 * NTPD shared memory reference clock driver structure
 */
#define SHMKEY 0x4e545030
struct shmTime {
	int     mode;
	int     count;
	time_t  clockTimeStampSec;
	int     clockTimeStampUSec;
	time_t  receiveTimeStampSec;
	int     receiveTimeStampUSec;
	int     leap;
	int     precision;
	int     nsamples;
	int     valid;
	int     dummy[10];
};

/*
 * Holds all the state information about a clock receiver
 */
struct clockInfo {
	int count;
	int status;
	int error;
	int frame;
	int correct;
	unsigned char marker;
	struct timeval start;
	struct timeval end;
	int unit;
	time_t last;
	struct shmTime *stamp;
	char line[4];
	char code[128];
	struct timeval pulses[128];
};


/*
 * Globals, no less
 */
int serial;
int poll;
int test;
jmp_buf saved;
struct clockInfo dcd,cts,dsr;


enum { MSF=0x01, DCF77=0x02, WWVB=0x04, JJY=0x08 };
enum { LEAP_NOWARNING=0x00, LEAP_NOTINSYNC=0x03};


/* Accuracy is assumed to be 2^PRECISION seconds -10 is approximately 980uS */
#define PRECISION (-10)

#define VERSION_STRING "\
radioclkd version 1.0\n\
Copyright (c) 2001-03 Jonathan A. Buzzard <jonathan@buzzard.org.uk>\n"

#define USAGE_STRING "\
Usage: radioclkd [-t] [-p] device\n\
Decode the time from a radio clock(s) attached to a serial port\n\n\
  -t,--test     print pulse lengths and times to stdout\n\
  -p,--poll     poll the serial port instead of using interrupts\n\
  -h,--help     display this help message\n\
  -v,--version  display version\n\
Report bugs to jonathan@buzzard.org.uk\n"


/*
 * Like mktime but ignores the current time zone and daylight savings, expects
 * an already normalized tm stuct, and does not recompute tm_yday and tm_wday.
 */
time_t UTCtime(struct tm *timeptr)
{
	int bits,direction,secs;
	struct tm search;
	time_t timep;


	/* calculate the number of magnitude bits in a time_t */
	for (bits=0,timep=1;timep>0;bits++,timep<<=1)
		;

	/* if time_t is signed, 0 is the median value else 1<<bits is median */
	timep = (timep<0) ? 0 : ((time_t) 1<<bits);

	/* save the seconds, and take them out of the search */
	secs = timeptr->tm_sec;
	timeptr->tm_sec = 0;

	/* binary search of the time space using the system gmtime() function */
	for (;;) {
	        search = *gmtime(&timep);

		/* compare the two times down to the same day */
		if (((direction = (search.tm_year-timeptr->tm_year))==0) &&
		    ((direction = (search.tm_mon-timeptr->tm_mon))==0))
			direction = (search.tm_mday-timeptr->tm_mday);

		/* compare the rest of the way if necesary */
		if (direction==0) {
			if (((direction = (search.tm_hour-timeptr->tm_hour))==0) &&
			    ((direction = (search.tm_min-timeptr->tm_min))==0))
				direction = search.tm_sec-timeptr->tm_sec;
		}

		/* is the search complete? */
		if (direction==0) {
			timeptr->tm_sec	= secs;
			return timep+secs;
		} else {
			if (bits--<0)
				return -1;
			if (bits<0)
				timep--;
			else if (direction>0)
				timep -= (time_t) 1 << bits;
			else
				timep += (time_t) 1 << bits;
		}
	}

	return -1;
}


/*
 * Decode the DCF77 signal. Return time since epoc on success, -1 on error.
 *
 * Note: We shift time from CET to UTC which is more useful for our purposes
 */
time_t DecodeDCF77(char *code, int length)
{
	int bcd[] = { 4,3,1,4,2,1,4,2,3,4,1,4,4 };
	int parity[] = { 8,7,23 };
	int segment[13];
	int i,j,k,sum;
	struct tm decoded;


	/* check the parity bits */
	k = length-38;
	for(i=0;i<3;i++) {
		sum = 0;
		for(j=0;j<parity[i];j++,k++)
			sum += code[k];
		if ((sum%2)!=0)
			return -1;
	}

	/* calculate all the individual BCD segments */
	k = length-38;
	for(i=0;i<13;i++) {
		sum = 0;
		for(j=0;j<bcd[i];j++,k++)
			sum += ((code[k]==1) ? 1 : 0) << j;
		segment[i] = sum;
	}

	/* decode the BCD segments into the time */
	decoded.tm_year = 100+segment[11]+(segment[12]*10);
	decoded.tm_mon = segment[9]+(segment[10]*10)-1;
	decoded.tm_mday = segment[6]+(segment[7]*10);
	decoded.tm_wday = segment[8];
	if (decoded.tm_wday==7)
		decoded.tm_wday = 0;
	decoded.tm_hour = segment[3]+(segment[4]*10);
	decoded.tm_min = segment[0]+(segment[1]*10);
	decoded.tm_sec = 0;
	decoded.tm_isdst = 0;

	/* some extra sanity checks */
	if ((decoded.tm_min>59) || (decoded.tm_hour>23) ||
			(decoded.tm_wday>6) || (decoded.tm_mday>31) ||
			(decoded.tm_mon>11) ||(decoded.tm_year>199))
		return -1;

	/* return adjusted for CET and DST */
	return (UTCtime(&decoded)-((code[length-42]==1) ? 7200 : 3600));
}


/*
 * Decode the MSF signal. Return time since epoc on success, -1 on error.
 */
time_t DecodeMSF(char *code, int length)
{
	int bcd[] = { 4,4,1,4,2,4,3,2,4,3,4 };
	int parity[] = { 8,11,3,13 };
	int segment[11];
	int i,j,k,sum;
	struct tm decoded;


	/* check the parity bits */
	k = length-44;
	for(i=0;i<4;i++) {
		sum = (code[length-7+i]==2) ? 1 : 0;
		for(j=0;j<parity[i];j++,k++)
			sum += code[k];
		if ((sum%2)!=1)
			return -1;
	}

	/* calculate all the individual BCD segments */
	k = length-44;
	for(i=0;i<11;i++) {
		sum = 0;
		for(j=0;j<bcd[i];j++,k++)
			sum += ((code[k]==1) ? 1 : 0) << (bcd[i]-j-1);
		segment[i] = sum;
	}

	/* decode the BCD segments into the time */
	decoded.tm_year = 100+(segment[0]*10)+segment[1];
	decoded.tm_mon = (segment[2]*10)+segment[3]-1;
	decoded.tm_mday = (segment[4]*10)+segment[5];
	decoded.tm_wday = segment[6];
	decoded.tm_hour = (segment[7]*10)+segment[8];
	decoded.tm_min = (segment[9]*10)+segment[10];
	decoded.tm_sec = 0;
	decoded.tm_isdst = 0;

	/* some extra sanity checks */
	if ((decoded.tm_min>59) || (decoded.tm_hour>23) ||
			(decoded.tm_wday>6) || (decoded.tm_mday>31) ||
			(decoded.tm_mon>11) ||(decoded.tm_year>199))
		return -1;

	/* return adjusted for daylight savings */
	return (UTCtime(&decoded)-((code[length-3]==2) ? 3600 : 0));
}


/*
 * Decode the WWVB signal. Return time since epoc on success, -1 on error.
 */
time_t DecodeWWVB(char *code, int length)
{
	int bcd[] = { 3,1,4,3,2,1,4,3,2,1,4,1,4,11,4,1,4 };
	int months[] = { 0,31,59,90,120,151,181,212,243,273,304,334 };
	int segment[17];
	int i,j,k,sum;
	struct tm decoded;


	/* check framing markers exist and data pulses are of correct type */
	for (i=2;i<60;i++) {
		j = code[length-i-1];
		k = (i-1)%10;
		if ((k==0) && (j!=5))
			return -1;
		else if ((k!=0) && (j!=1) && (j!=4))
			return -1;
	}

	/* calculate all the individual BCD segments */
	k = length-60;
	for(i=0;i<17;i++) {
		sum = 0;
		for(j=0;j<bcd[i];j++,k++)
			sum += ((code[k]==4) ? 1 : 0) << (bcd[i]-j-1);
		segment[i] = sum;
	}

	/* decode the BCD segments into the time */
	decoded.tm_year = 100+segment[16]+(segment[14]*10);
	decoded.tm_yday = segment[12]+(segment[10]*10)+(segment[8]*100)-1;
	decoded.tm_hour = segment[6]+(segment[4]*10);
	decoded.tm_min = segment[2]+(segment[0]*10);
	decoded.tm_sec = 0;
	decoded.tm_isdst = 0;

	/* some extra sanity checks */
	if ((decoded.tm_min>59) || (decoded.tm_hour>23) ||
			(decoded.tm_yday>365) || (decoded.tm_year>199))
		return -1;

	/* set the month and day of month fields */
	decoded.tm_mon = -1;
	for (i=11;i>=0;i--) {
		if (months[i]<=decoded.tm_yday) {
			decoded.tm_mon = i;
			decoded.tm_mday = 1+decoded.tm_yday-months[i];
			break;
		}
	}

	/* adjust for leap years */
	if (code[length-6]==4) {
		if (decoded.tm_yday>59) {
			decoded.tm_mday--;
		} else if (decoded.tm_yday==59) {	
			decoded.tm_mon = 1;
			decoded.tm_mday = 29;
		}
	}
	if (decoded.tm_mon==-1)
		return -1;

	/* WWVB transmits the time for the minute just gone so adjust */
	return (UTCtime(&decoded)+60);
}


/*
 * Print the pulse information
 */
void PrintPulseInfo(struct clockInfo *c)
{
	struct timeval tv;

	timersub(&c->end, &c->start, &tv);
	fprintf(stdout, "%s: %3d %4d %9d   ", c->line, c->count,
		c->code[c->count-1], (int) tv.tv_usec);

	return;
}


/*
 * Log a warning if no signal has been received in the last 5 minutes
 */
void LogNoSignalWarning(struct clockInfo *c, time_t now)
{
	if (((now-c->last)>300) && (c->last>-1) && (c->error==0)) {
		c->error++;
		syslog(LOG_INFO, "no valid time received in last five minutes "
			"for %s line", c->line);
	}

	return;
}


/*
 * Attach the shared memory segment for the reference clock driver
 */
struct shmTime *AttachSharedMemory(int unit, int *shmid)
{
	struct shmTime *shm;

	*shmid = shmget(SHMKEY+unit, sizeof(struct shmTime), IPC_CREAT | 0700);
	if (*shmid==-1)
		return NULL;

	shm = (struct shmTime *) shmat(*shmid, 0, 0);
	if ((shm==(void *) -1) || (shm==0))
		return NULL;

	return shm;
}


/*
 * Set the DTR and RTS line to power the device(s) on.
 */
int TurnReceiverOn(int fd)
{
	int arg;

	if (ioctl(fd, TIOCMGET, &arg)!=0)
		return -1;
	arg |= (TIOCM_DTR | TIOCM_RTS);
	if (ioctl(fd, TIOCMSET, &arg)!=0)
		return -1;

	return 0;
}


/*
 * Time out handler for the alarm on TIOCMIWAIT
 */
void SerialTimeoutAlarm(int sig)
{
//	signal(SIGALRM, SerialTimeoutAlarm);
//	fprintf(stderr, "ALARM SIGNAL RECIEVED\n");
	longjmp(saved, 1);

	return;
}


/*
 * Wait till either the DCD, CTS or DSR line changes status on the serial port
 */
int WaitOnSerialChange(int fd, struct timeval *tv)
{
	int i,arg,cts,dcd,dsr;


	/* loop polling for the DCD, CTS or DSR line to change status */
	if (poll==1) {
		if (ioctl(fd, TIOCMGET, &arg)!=0)
			return -1;
		dcd = arg & TIOCM_CD;
		cts = arg & TIOCM_CTS;
		dsr = arg & TIOCM_DSR;
		for (i=0;i<2000;i++) {
			usleep(5000);
			if (ioctl(fd, TIOCMGET, &arg)!=0)
				return -1;
			gettimeofday(tv, NULL);
			if ((dcd!=(arg & TIOCM_CD)) || (cts!=(arg & TIOCM_CTS))
			    || (dsr!=(arg & TIOCM_DSR)))
				return arg;
		}
		/* nothing changed for 10 seconds return with error */
		return -1;
	}

	/* set a timeout for TIOCMIWAIT */
	if (setjmp(saved)!=0)
		return -1;
	signal(SIGALRM, SerialTimeoutAlarm);
	alarm(10);

	/* wait till a serial port status change interrupt is generated */
	if (ioctl(fd, TIOCMIWAIT, TIOCM_CD | TIOCM_CTS | TIOCM_DSR)!=0)
		return -1;
	gettimeofday(tv, NULL);
	if (ioctl(fd, TIOCMGET, &arg)!=0)
		return -1;

	/* cancel the timeout */
	alarm(0);

	return arg;
}


/*
 * Place a time stamp in the SHM segment for the NTP reference clock driver
 */
void PutTimeStamp(struct timeval *local, struct timeval *radio,
	struct shmTime *shm, int leap)
{
	shm->mode = 1;
	shm->valid = 0;

	__asm__ __volatile__ ("":::"memory");

	shm->leap = leap;
	shm->precision = PRECISION;
	shm->clockTimeStampSec = (time_t) radio->tv_sec;
	shm->clockTimeStampUSec = (int) radio->tv_usec;
	shm->receiveTimeStampSec = (time_t) local->tv_sec;
	shm->receiveTimeStampUSec = (int) local->tv_usec;

	__asm__ __volatile__ ("":::"memory");

	shm->count++;
	shm->valid = 1;

	return;
}


/*
 * Time comparison routine for the C library quicksort routine
 */
static int TimeCompare(const void *a, const void *b)
{
	int timea,timeb;

	timea = *(int *) a;
	timeb = *(int *) b;

	if (timea<timeb)
		return -1;
	else if (timea>timeb)
		return +1;
	else
		return 0;
}


/*
 * Calculate the average measured offset of the start of the radioclock
 * pulses from the true time over the last minute
 */
int CalculatePPSAverage(struct clockInfo *c, int *average)
{
	int i,err,count;
	long sum;
	int timediff[59] = { 0 };

	/* this only works if we have a full minutes worth of clock pulses */
	if (c->count<59)
		return -1;

	/* calculate the measured clock offset for the start of each pulse */
	for (i=0;i<59;i++) {
		/* calculate time difference between computer and radio
		   for each second marker */
		err = (int) c->pulses[c->count-i-1].tv_usec;
		if (err>500000)
			err -= 1000000;
			
		/* if the time isn't close, don't bother tracking it */
		if (abs(err)>128000)
			return -1;

		timediff[i] = err;
	}

	/* now sort them into order */
	qsort(timediff, 59, sizeof(long), TimeCompare);

	/* calculate the arithmetic mean of the middle half */
	count = 0;
	sum = 0;
	for (i=15;i<45;i++) {
		sum += timediff[i];
		count++;
	}
	*average = (int) sum/count;

	return 0;
}


/*
 * Process a received time code and place stamp into shared memory
 */
void ProcessTimeCode(struct clockInfo *c, int radio)
{
	time_t decoded,last;
	struct timeval computer,received;
	int i,shmid,average;


	/* decode the time */
	switch (radio) {
		case DCF77:
			decoded = DecodeDCF77(c->code, c->count);
			break;
		case MSF:
			decoded = DecodeMSF(c->code, c->count);
			break;
		case WWVB:
			decoded = DecodeWWVB(c->code, c->count);
			break;
		default:
			c->count = 1;
			c->marker = 0x00;
			c->frame = 0;
			c->correct = 0;
			return;
	}

	/* place time stamp into shared memory segment or print on stdout */	
	if (test==0) {
		/* final sanity check on the time */
		if (abs(c->start.tv_sec-decoded)>1000) {
			syslog(LOG_INFO, "decoded time differs from system "
				"time by more than 1000s ignored");
			c->count = 1;
			c->marker = 0x00;
			c->frame = 0;
			c->correct = 0;
			return;
		}

		/* attach shared memory segment if not already done */
		if (c->stamp==NULL) {
			c->stamp = AttachSharedMemory(c->unit, &shmid);
			if ((shmid==-1) || (c->stamp==NULL)) {
				syslog(LOG_INFO, "unable to attach shared "
					"memory for %s", c->line);
					return;
			}
		}

		/* if possible use an averaged offset */
		if (CalculatePPSAverage(c, &average)<0) {
			computer.tv_sec = c->start.tv_sec;
			computer.tv_usec = c->start.tv_usec;
		} else {
			if (average<0) {
				computer.tv_sec = decoded-1;
				computer.tv_usec = average+1000000;
			} else {
				computer.tv_sec = decoded;
				computer.tv_usec = average;
			}
		}
		
		/* put time stamp in shared memory segment for ntpd */
		received.tv_sec = decoded;
		received.tv_usec = 0;
		PutTimeStamp(&computer, &received, c->stamp, LEAP_NOWARNING);

		/* log any errors in getting the time */
		last = decoded-c->last;
		if ((last>3600) && (c->error>0)) {
			syslog(LOG_INFO, " %ldh %ldm since previous valid time "
				"for %s line", last/3600, (last%3600)/60,
				c->line);
		} else if ((last>300) && (c->error>0)) {
			syslog(LOG_INFO, " %ldm since previous valid time for %s"
				" line", last/60, c->line);
		}
	} else {
		/* any valid time is printed in testing mode */
		for (i=1;i<c->count;i++)
			fprintf(stdout, "%1d", c->code[i]);
		fprintf(stdout, "\nUTC: %s", ctime(&decoded));
	}

	/* reset the error warning and set last stamp time */
	c->error = 0;
	c->last = decoded;

	/* setup for receiving the next minute of pulses */
	c->count = 1;
	c->marker = 0x00;
	c->frame = 0;
	c->correct = 0;

	return;
}


/*
 * Process a status change on the serial port to calculate the pulse type.
 * If a minute marker is present send time stamp to ntpd.
 */
void ProcessStatusChange(struct clockInfo *c, int arg, struct timeval *tv)
{
	struct timeval length;

	if ((!arg) && (c->status==1)) {
		c->status = 0;
		c->start.tv_sec = tv->tv_sec;
		c->start.tv_usec = tv->tv_usec;
		timersub(&c->start, &c->end, &length);
		/* check for the DCF77 minute marker */
		if ((length.tv_sec==1) && (length.tv_usec>=760000) &&
				(length.tv_usec<=950000) && (c->count>44)) {
			
			c->pulses[c->count].tv_sec = c->start.tv_sec;
			c->pulses[c->count].tv_usec = c->start.tv_usec;
			ProcessTimeCode(c, DCF77);
			return;
		}

		/* check to see if bit B of the MSF code set */
		if ((length.tv_usec>=60000) && (length.tv_usec<=150000)) {
			c->code[c->count-1] = 3;
			c->correct = 1;
		}

	} else if ((arg) && (c->status==0)) {
		c->status = 1;
		c->end.tv_sec = tv->tv_sec;
		c->end.tv_usec = tv->tv_usec;
		timersub(&c->end, &c->start, &length);

		if (c->correct==1) {
			/* make a correction for MSF bit B being set */
			c->correct = 0;
			return;			
		} else if ((length.tv_usec>=60000) && (length.tv_usec<150000)) {
			c->code[c->count] = 0;
			c->pulses[c->count].tv_sec = c->start.tv_sec;
			c->pulses[c->count].tv_usec = c->start.tv_usec;
			c->count++;
			c->frame = 0;
			c->marker = c->marker<<1;
		} else if ((length.tv_usec>=160000) && (length.tv_usec<250000)) {
			c->code[c->count] = 1;
			c->pulses[c->count].tv_sec = c->start.tv_sec;
			c->pulses[c->count].tv_usec = c->start.tv_usec;
			c->count++;
			c->frame = 0;
			c->marker = (c->marker<<1) | 1;
		} else if ((length.tv_usec>=260000) && (length.tv_usec<350000)) {
			c->code[c->count] = 2;
			c->pulses[c->count].tv_sec = c->start.tv_sec;
			c->pulses[c->count].tv_usec = c->start.tv_usec;
			c->count++;
			c->frame = 0;
			c->marker = (c->marker<<1) | 1;
		} else if ((length.tv_usec>=460000) && (length.tv_usec<550000)) {
			c->code[c->count] = 4;
			c->pulses[c->count].tv_sec = c->start.tv_sec;
			c->pulses[c->count].tv_usec = c->start.tv_usec;
			c->count++;
			c->frame = 0;
			/* check for MSF minute marker */
			if ((c->marker==0x7e) && (c->count>42)) {
				ProcessTimeCode(c, MSF);
				return;
			}
		} else if ((length.tv_usec>=760000) && (length.tv_usec<850000)) {
			c->code[c->count] = 5;
			c->pulses[c->count].tv_sec = c->start.tv_sec;
			c->pulses[c->count].tv_usec = c->start.tv_usec;
			c->count++;
			c->frame++;
			/* check for the WWVB minute marker */
			if ((c->frame==2) && (c->count>60)) {
				ProcessTimeCode(c, WWVB);
				return;
			}			
		} else {
			/* unknown pulse must be an error reset */
			c->count = 1;
			c->marker = 0x00;
			c->frame = 0;
			c->correct = 0;
		}

	}

	/* check for missing minute marker and reset if needed */
	if (c->count==128) {
		c->count = 1;
		c->marker = 0x00;
		c->frame = 0;
		c->correct = 0;
	}

	return;
}


/*
 * Catch any signals sent, and exit cleanly.
 */
void Catch(int sig)
{
	if (test==0) {
		syslog(LOG_INFO, "Exiting...");
		unlink(PID_FILE);
		munlockall();
		if (dcd.stamp!=NULL)
			shmdt(dcd.stamp);
		if (cts.stamp!=NULL)
			shmdt(cts.stamp);
		if (dsr.stamp!=NULL)
			shmdt(dsr.stamp);
	} else {
		fprintf(stderr, "radioclkd: Exiting...\n" );
	}
	close(serial);

	exit(0);
}


/*
 * Entry point.
 */
int main(int argc, char *argv[]) 
{
	int i,pid,arg;
	struct sched_param schedp;
	struct timeval tv;
	time_t now;
	FILE *str;
	char devname[16] = "/dev/";


	/* process the command line arguments */
	poll = 0;
	test = 0;
	for (i=1;i<argc;i++) {
		if ((!strcmp(argv[i], "-h")) || (!strcmp(argv[i], "--help"))) {
			fprintf(stdout, USAGE_STRING);
			exit(0);
		} else if ((!strcmp(argv[i], "-v")) || (!strcmp(argv[i], "--version"))) {
			fprintf(stdout, VERSION_STRING);
			exit(0);
		} else if ((!strcmp(argv[i], "-p")) || (!strcmp(argv[i], "--poll"))) {
			poll = 1;
		} else if ((!strcmp(argv[i], "-t")) || (!strcmp(argv[i], "--test"))) {
			test = 1;
			/* switch timezone to UTC so time functions do right thing */
			putenv("TZ=''");
		} else {
			strncat(devname, argv[i], 10);
		}
	}
			
	if (strlen(devname)<6) {
		fprintf(stderr, "radioclkd: error no serial port specified\n");
		return 1;
	}

	/* open the serial port */
	if ((serial = open(devname, O_RDWR | O_NOCTTY | O_NDELAY))<0) {
		fprintf(stderr, "radioclkd: couldn't open device %s\n", devname);
		return 1;
	}

	/* register some signal handlers */
	if (signal(SIGINT, SIG_IGN)!=SIG_IGN)
		signal(SIGINT, Catch);
	if (signal(SIGQUIT, SIG_IGN )!=SIG_IGN)
		signal(SIGQUIT, Catch);
	if (signal(SIGTERM, SIG_IGN)!=SIG_IGN)
		signal(SIGTERM, Catch);
	signal(SIGUSR1, SIG_IGN);

	/* power up the receiver(s) */
	if (TurnReceiverOn(serial)!=0) {
		fprintf(stderr, "radioclkd: error powering up receiver\n");
		close(serial);
		return 1;
	}

	/* check to see if a copy of radioclkd is already running */
	if (!access(PID_FILE, R_OK)) {
		if ((str = fopen(PID_FILE, "r" ))) {
			fscanf(str, "%d", &pid);
			fclose(str);

			/* check the other radioclkd is still running */
			if (kill(pid, SIGUSR1)==0) {
				fprintf(stderr, "radioclkd: Already running as "
					"process %d.\n", pid);
				return 1;
			}

			fprintf(stderr, "radioclkd: process %d appears to have "
				"died, continuing\n", pid);
			unlink(PID_FILE);
		}
	}

	/* do things specific to the daemon version */
	if (test==0) {
		/* open connection to system logger */
		openlog("radioclkd", LOG_PID | LOG_CONS, LOG_DAEMON);

		/* now looks like a good time to become a daemon */

 		/* parent */
 		if ((pid=fork())) {
 			if ((str=fopen(PID_FILE, "w"))) {
 				fprintf(str, "%d\n", pid);
 				fclose(str);
 			}
 			close(serial);
 			return 0;
 		}
 
 		/* child */
 		if (pid!=0) {
 			syslog(LOG_INFO, "fork() failed: %m");
 			unlink(PID_FILE);
 			close(serial);
 			return 1;
 		} else {
 			syslog(LOG_INFO, "entering daemon mode");
 		}
 	
 		/* child - Follow the daemon rules in W. Richard Stevens.
 		   Advanced Programming in the UNIX Environment (Addison-Wesley
 		   Publishing Co., 1992). Page 417.). */
 		if (setsid()<0) {
 			syslog(LOG_INFO, "setsid() failed: %m");
 			unlink(PID_FILE);
 			close(serial);
 			return 1;
 		}

		/* set realtime scheduling priority */
		memset(&schedp, 0, sizeof(schedp));
		schedp.sched_priority = sched_get_priority_max(SCHED_FIFO);	
		if (sched_setscheduler(0, SCHED_FIFO, &schedp)!=0)
			syslog(LOG_INFO, "error unable to set real time "
				"scheduling");

		/* lock all memory pages */
		if (mlockall(MCL_CURRENT | MCL_FUTURE) !=0)
			syslog(LOG_INFO, "error unable to lock memory pages");
	
	}

	/* pause a few seconds to allow receiver(s) to power up */
	sleep(5);

	/* some safety precautions */
	chdir("/");
	umask(0);

	/* initialize the three clock structures */
	memset(&dcd, 0, sizeof(struct clockInfo));
	memset(&cts, 0, sizeof(struct clockInfo));
	memset(&dsr, 0, sizeof(struct clockInfo));
	dcd.count = cts.count = dsr.count = 1;
	dcd.last = cts.last = dsr.last = -1;
	dcd.unit = 0;
	cts.unit = 1;
	dsr.unit = 2;
	strcpy(dcd.line, "DCD");
	strcpy(cts.line, "CTS");
	strcpy(dsr.line, "DSR");

	/* loop  until we die */
	for (;;) {
		arg = WaitOnSerialChange(serial, &tv);

		/* first process any clock on the DCD status line */
		ProcessStatusChange(&dcd, (arg & TIOCM_CD), &tv);

		/* now do the same for a clock on the CTS line */
		ProcessStatusChange(&cts, (arg & TIOCM_CTS), &tv);

		/* now do the same for a clock on the DSR line */
		ProcessStatusChange(&dsr, (arg & TIOCM_DSR), &tv);

		/* print pulse information on stdout if in test mode */
		if ((test==1) && ((dcd.status==1) || (cts.status==1) || (dsr.status==1))) {
			PrintPulseInfo(&dcd);
			PrintPulseInfo(&cts);
			PrintPulseInfo(&dsr);
			fprintf(stdout, "\n");
		}

		/* warn if valid time stamp not received in the last 5 mins */
		time(&now);
		LogNoSignalWarning(&dcd, now);
		LogNoSignalWarning(&cts, now);
		LogNoSignalWarning(&dsr, now);
	}

	return 0;
}

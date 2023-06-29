/*************************************************************************************************************
 * Stylo.c this is code for raspberry pi pico RP2040. It is the  14 oscillator Picodrone.
 *
 *
 * 14 Bit D to A pins 0-13
 * Pins 14-17 are MUX drivers
 * Pins 18-21 switch pins with a diode and MUX pins this gives 16 switches
 * Pin  22 is the LED pin. With the MUX drivers this controls 4 LEDS (they can even share a single current
 * limiting resistor)
 * Pins 23-24-25-26 are internal to the Pico									
 * Pin  26-27-28 are set to ADC for three pots								
 *
 * Martin Parker	-	The Inner Loop
 * Version		-	0.1 First complete version with 
 * 
 *
 * 	Unreliable Sequencer
 * 	Triangle and Saw waves
 * 	Wave shaper to square  with variable level
 * 	Octave twiddler
 * 	Delay and reverb
 * 	Crude filter
 * 	Waveform random crusher 
 * 	45455 Samples per second output rate
 * 	16 Keys and 4 LEDS multiplexed
 *
 *
 **********************************************************************************************************/  

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/divider.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "pico/util/queue.h"
#define D1 14
#define D2 15
#define D3 16
#define D4 17
#define S1 18
#define S2 19
#define S3 20 
#define S4 21 
#define LED 22 

// sequencer events
struct event { uint64_t time[400]; int key[400]; int onoff[400];};

// global variables between knobs and oscillators
int trem,filt,latch,del,amp[14],voice,rev,oct,cut,peak;

// convert recoded events into playable events
int retime (struct event *e, struct event *f,uint64_t start, int speed, int rel)
{
	int i;
	speed+=2048;
	for (i=0;i<100;i++)
	{
		f->time[i]=(e->time[i]*4096/(uint64_t)speed)+start;
		f->key[i]=e->key[i];
		// unrelaible?
		if (rand()%4097<rel) { f->onoff[i]=e->onoff[i];}else{f->onoff[i]=0;}
	}
}

// 14 bit D to A. Add 8192 to give 0-16384 range from +/- 8192 input
int d_to_a ( int val)
{
	uint32_t set,value;
	static uint32_t mask;
	int i;

	mask=16383;
	value=0;

	val+=8191;
	if (val<0 || val >16383){latch=5000;}

	for (i=0;i<14;i++)
	{
		value=val&1;
		value<<=1;val>>=1;

	}
	gpio_put_masked	(mask,value);
/*
	gpio_put(13,val&1);
	gpio_put(12,val&2);
	gpio_put(11,val&4);
	gpio_put(10,val&8);
	gpio_put(9,val&16);
	gpio_put(8,val&32);
	gpio_put(7,val&64);
	gpio_put(6,val&128);
	gpio_put(5,val&256);
	gpio_put(4,val&512);
	gpio_put(3,val&1024);
	gpio_put(2,val&2048);
	gpio_put(1,val&4096); */

}

// this controls all the knobs and switches and LEDS.
void knobs_thread()
{
    uint64_t delay,wanted,rec_start;
    int record,pot1l,pot2l,pot3l,pot1n,pot2n,pot3n,tick,tremd;

    static uint64_t outrate=30000;

    // the delay in microseconds for each loop. It needs to be constant so that the ADSR etc is consistent
    delay=1000000/outrate;

    adc_init(); adc_gpio_init(26); adc_gpio_init(27); adc_gpio_init(28);

    struct event *e,*f;
    e=(struct event *)malloc(sizeof(struct event));
    f=(struct event *)malloc(sizeof(struct event));

    int a,d,amps[14],leds[4],flash,flashc,led,seqc,speed,rel,att,dec,sus;
    int wwd,i,attack[14],sm,switches[20],press[20],pot1,pot2,pot3,bounce;
    int filtl,mode,octs,wiggle,wd,fp,hold,play,playp,octl,oc,octf,ood,od;
     a=0;d=0;tremd=0;
    voice=0; rev=0; sm=0; seqc=0; octl=0;oc=0;octf=0;
    tick=0; mode=0; wd=0; wwd=wd; filtl=132000; record=0;wiggle=0;
    od=0;ood=od;
    latch=0;
    for (i=0;i<14;i++){amp[i]=0;amps[i]=0;attack[i]=0;switches[i]=0;press[i]=0;}
    for (i=0;i<4;i++){leds[i]=1;}
    leds[0]=0;flash=0;flashc=0;led=0;playp=0;
    oct=48; octs=48;cut=65537; peak=65535; del=10000; bounce=0;hold=0;play=0;speed=2048;rel=4096;
    att=1;dec=1;sus=4096;oc=0;


    sleep_ms(400);
    wanted=time_us_64();
    while (1)
    {
   	wanted+=delay;
	int count;
	count=0;
	while (time_us_64()<wanted)
	{
		sleep_us(1);
		count++;
	}
	// overload
	if (count<3) {latch=5000;}
	
	if(latch){ latch--;leds[0]=0;leds[1]=0;leds[2]=0;leds[3]=0;} else{
		//led handler
		flashc++;if (flashc>16000){flashc=0;}
		if (flash && flashc<(9000/flash)){ led=1;}else{led=0;}
	}

	// switch mux
	// S1 does 0 1 2 3 S2 4 5 6 7 S3 8 9 10 11 S4 12 13 14 15
	// its a bit backwards set it for the next one, this means my one is settled before I measure it.
	switch (sm)
	{
	    case 0 : 
		/*gpio_put(D4,0); 
		gpio_put(LED,leds[0]);
		gpio_put(D1,1); sleep_us(1);*/

		switches[0]=gpio_get(S1);
		switches[4]=gpio_get(S2);
		switches[8]=gpio_get(S3);
		switches[12]=gpio_get(S4);

		gpio_put(D1,0); 
		gpio_put(LED,leds[1]);
		gpio_put(D2,1); 
		break;

	    case 1 :	
		switches[1]=gpio_get(S1);
		switches[5]=gpio_get(S2);
		switches[9]=gpio_get(S3);
		switches[13]=gpio_get(S4);

		gpio_put(D2,0); 
		gpio_put(LED,leds[2]);
		gpio_put(D3,1); 
		break;

	    case 2 :	
		switches[2]=gpio_get(S1);
		switches[6]=gpio_get(S2);
		switches[10]=gpio_get(S3);
		switches[14]=gpio_get(S4);

		gpio_put(D3,0); 
		gpio_put(LED,leds[3]);
		gpio_put(D4,1); 
		break;


	   case 3 :	
		switches[3]=gpio_get(S1);
		switches[7]=gpio_get(S2);
		switches[11]=gpio_get(S3);
		switches[15]=gpio_get(S4);

		gpio_put(D4,0); 
		gpio_put(LED,leds[0]);
		gpio_put(D1,1); 

		sm=-1;
		break;
	}
    	sm++;
	
	// filter wiggle
	wiggle+=wd;
	if (wiggle>819200){ wiggle=819200;wd=-wwd;}
	if (wiggle<0){ wiggle=0;wd=wwd;}

	// octave wiggle
	oc+=od;
	if (oc>480000){ oc=480000;od=-ood;}
	if (oc<-480000){ oc=-480000;od=ood;}


	//adc 
    	adc_select_input(0);
       	pot1n = (int)adc_read(); pot1=(pot1n+(99*pot1l))/100; pot1l=pot1; 
     	adc_select_input(1);
       	pot2n = (int)adc_read(); pot2=(pot2n+(99*pot2l))/100; pot2l=pot2; 
     	adc_select_input(2);
       	pot3n = (int)adc_read(); pot3=(pot3n+(99*pot3l))/100; pot3l=pot3;

	// normalise the amplitudes
	int tot;
	tot=0;
	for (i=0;i<14;i++) { tot+=amps[i]; }
	if (tot>16384){ 
		for (i=0;i<14;i++) { amp[i]=amps[i]*16384/tot; }
	}
	else{
		for (i=0;i<14;i++) { amp[i]=amps[i];}
	}

	// playback
	if (play)
	{
		if (f->time[playp]<wanted)
		{
			switches[f->key[playp]]=f->onoff[playp];
			leds[2]=0;
			playp++;
		}
		if (playp>=seqc)
		{
			playp=0;
			retime(e,f,wanted,speed,rel); 
			leds[2]=1;
		}

	}

	if (a>=att){ a=0;}
	if (d>=dec){ d=0;}
	for (i=0;i<14;i++)
	{	
		if (switches[i] )
		{
			if (!press[i]){ attack[i]=1;press[i]=1;amps[i]=0;
				// sequencer
				if (record){e->time[seqc]=wanted-rec_start;e->key[seqc]=i;e->onoff[seqc]=1;seqc++;}
			}
			// hold activted. switch of 15 so no other action happens
			if (switches[15]){attack[i]=5; switches[15]=0;}
			// unhold everything
			if (switches[14]){ int j; for (j=0;j<14;j++){attack[j]=4; switches[14]=0;} break;}
		} else { press[i]=0;}
		
	
		// ADSR	
		// 1 Attack grow from 0 to 8191
		if (attack[i]==1 && !a)
		{
			if (amps[i]<8191 ){ amps[i]+=4;}else{amps[i]=8191;attack[i]=2;}
		// 2 decay drop to half 
		} else if ( attack[i]==2 && !d) {
 			if (amps[i]>sus ){ amps[i]-=4;}else{attack[i]=3;}
		// 3 sustain stay at half until key released 
		} else if (attack[i]==3) {
			if ( !switches[i]){ attack[i]=4;
				if (record){ e->time[seqc]=wanted-rec_start;e->key[seqc]=i;e->onoff[seqc]=0;seqc++;}
			}
		// 4 release mode
		} else if (attack[i]==4) {
 			if (amps[i]>0 ){ amps[i]-=5;}else{attack[i]=0;amps[i]=0;}
		// 5 special held mode for droning
		} else if (attack[i]==5) { amps[i]=4096;}
	}
	a++;d++;oc++;
	// mode switches
	if (switches[14] && !press[14] ){ press[14]=1;}
	// release 14
	if (!switches[14] && press[14] ){ 
		press[14]=0; 
		switch (mode)
		{	
			case 0 : octs+=12; if (octs>60){octs=24;}  break;
			case 1 : voice=1-voice; break;
			case 2 : rev++;if (rev>2){rev=0;} break;
			case 3 : break;
			case 4 :	
				if (!record && !play){flash=3;record=1;seqc=0;rec_start=wanted;}
				else if (record){flash=5;record=0;
					if (seqc>0){ play=1; retime(e,f,wanted,2048,4096); playp=0;} else{ flash=0;} 
				}else if (play){ flash=0;play=0;}
				break;
		}
	}
	// in bounce mode switch 15 is disabled for a while.
	if (bounce){ switches[15]=0;bounce++;if (bounce>5000){bounce=0;}}
	if (switches[15] && press[15] ) { hold++;if (hold>9000){flash=1;}}
	if (switches[15] && !press[15] ) { press[15]=1;hold=0;flash=0;}
	// key 15 is being held on
	// released 
	if (!switches[15] && press[15] ) { 
		press[15]=0;
		// not flashing change mode
		if (!flash ) { mode++;if (mode>4){mode=0;}}
		bounce=1;
	}

	// this big switch is useful I bunged the LEDS in here
	// octaves 
	switch (mode)
	{
	// octaves
	case 0 : 
		leds[0]=led;leds[1]=1;leds[2]=1;leds[3]=1; 
		if (flash==1){
	 	octs=36+(pot1/100) ;
		wwd=pot2/10;
		ood=pot3/5; }
		break;
	// voices 
	case 1 :
		leds[0]=1;leds[1]=led;leds[2]=1;leds[3]=1; 
		if (flash==1) {
		cut=(pot1*17);
       		peak=65535-(31*pot2);
		filtl=pot3*4096;}
		break;

	// delay time 
	case 2  :
		leds[0]=1;leds[1]=1;leds[2]=led;leds[3]=1; 
		if (flash==1){
		del=(8*pot1)+10000;
		ood=pot2/5; 
		tremd=pot3/10; }
		break;
	// ADSR
	case 3 :	
		leds[0]=1;leds[1]=1;leds[2]=1;leds[3]=led; 
		if (flash==1){
		att=(pot1/100);
		dec=(pot2/100);
		sus=(pot3);}
		break;
	// sequencer
	case 4 :
		leds[0]=1;leds[1]=led;leds[2]=led;leds[3]=1;
		if (flash==1){
		speed=100+pot1;
		rel=pot2;}
		break;
	}
	filt=filtl;
	oct=octs;
	if (wwd<10){filt=filtl;wd=10;wiggle=0;}else{filt=filtl+(wiggle*10);}
	//if (ood<10){oct=octs;od=10;oc=0;}else{oct=octs+(oc/150000)-1;}
	if (ood<10){od=10;oc=0;trem=0;}else{trem=((oc/500)*tremd)/5000;}
    }
}

int init_pins ()
{
    	// D to A pins 0-13 
    	int i;
    	for (i=0;i<14;i++)
    	{
    		gpio_init(i); gpio_set_dir(i,GPIO_OUT);
		// the slew rate and pull resistors make little difference to the sound
		// gpio_disable_pulls(i);
		//gpio_set_slew_rate(i,GPIO_SLEW_RATE_SLOW);
    	}
    	gpio_init(D1); gpio_set_dir(D1,GPIO_OUT); //Driver 1 
    	gpio_init(D2); gpio_set_dir(D2,GPIO_OUT); //Driver 2 
    	gpio_init(D3); gpio_set_dir(D3,GPIO_OUT); //Driver 3 
    	gpio_init(D4); gpio_set_dir(D4,GPIO_OUT); //Driver 4 
    	gpio_init(LED); gpio_set_dir(LED,GPIO_OUT); // LED 
    	gpio_init(S1); gpio_set_dir(S1,GPIO_IN); gpio_pull_down(S1); //key 1
    	gpio_init(S2); gpio_set_dir(S2,GPIO_IN); gpio_pull_down(S2); //key 2
    	gpio_init(S3); gpio_set_dir(S3,GPIO_IN); gpio_pull_down(S3); //key 3
    	gpio_init(S4); gpio_set_dir(S4,GPIO_IN); gpio_pull_down(S4); //key 4
    	gpio_init(23); gpio_set_dir(23,GPIO_OUT); gpio_put(23,1); //PSU noise thing
}


int main()
{
	// adding stdio causes interrupts and slows everything down. Debug only.
    	//stdio_init_all();
	int *buffer;
	int i,buffer_size;


	// max 1 second buffer
	buffer_size=44000;
	buffer=(int *)malloc(sizeof(int)*buffer_size);
	if (buffer==NULL){exit (1);}

	init_pins();
	sleep_ms(300);

    	multicore_launch_core1(knobs_thread);

    	for (i=0;i<buffer_size;i++){ buffer[i]=0;}

    	uint64_t wait,wanted;
	int ticker;
	ticker=1;

	//22 is 45455 Samples per sec.
	wait=22;
	int buff_point,delay_point;
	
	static int slews[]={45,47,50,53,56,59,63,67,71,75,79,84,89,94,100,106,112,119,126,133,141,150,159,168,178,189,200,212,224,238,252,267,283,299,317,336,356,377,400,423,449,475,503,533,565,599,634,672,712,754,799,847,897,950,1007,1067,1130,1198,1269,1344,1424,1509,1599,1694,1794,1901,2014,2134,2261,2395,2538,2688,2848,3019,3198,3388,3590,3803,4028,4268,4521,4790,5075,5378,5698,6035,6393,6773,7177,7604,8057,8535,9043,9579,10150,10753,11393,12071,12789,12789,12789,12789,12789,12789,12789,12789,12789,12789,12789};
	int sig[14];
	int sigp[14];
	int tl,fp,sigt_p;
	int slew[109];
	fp=0;sigt_p=0;

	for (i=0;i<14;i++){ sig[i]=0;sigp[i]=0;}
	for (i=0;i<109;i++){slew[i]=slews[i];}

    	wanted=time_us_64();
 	while (1)
 	{
    		wanted+=wait;
		short count;
		count=0;
		while (time_us_64()<wanted)
		{
			sleep_us(1);
			count++;
		}
		if (count<3) { latch=5000;} 

		int sig_tot;
		sig_tot=0;
		if (voice)
		{
			for (i=0;i<14;i++){ 
				sig[i]+=slews[i+oct]+trem;
				if (sig[i]>65535){sig[i]-=131072;}
				int t;
				t=sig[i];
				// thresh
				if (t>cut ){ t=peak;}
				if (t<-cut){ t=-peak;}

				if (amp[i]<10){sig[i]=0;}

				sig_tot+=t*amp[i];
			}
		}else{
			for (i=0;i<14;i++){ 
				int tls;
				tls=slew[i+oct]+trem;
				sig[i]+=tls;
				if (sig[i]>65535){sig[i]=131072-sig[i];slew[i+oct]=-tls;}
				if (sig[i]<-65535){sig[i]=-131072-sig[i];slew[i+oct]=-tls;} 
				int t;
				t=sig[i];
				//thresh
				if (t>cut ){ t=peak;}
				if (t<-cut){ t=-peak;}

				if (amp[i]<10){sig[i]=0;}
				sig_tot+=t*amp[i];
			}
		}

		int df;
		df=sig_tot-sigt_p;
		if (df>filt){ sig_tot=sigt_p+filt;}
		if (df<-filt){ sig_tot=sigt_p-filt;}
		sigt_p=sig_tot;

		if (rev==1)
		{
			buff_point++;if (buff_point>buffer_size){buff_point=0;}
			delay_point=buff_point-(del);if (delay_point<0){delay_point+=buffer_size;}
			buffer[buff_point]=sig_tot;
			d_to_a(((sig_tot+buffer[delay_point])/262072));}
		else if (rev==2)
		{
			buff_point++;if (buff_point>buffer_size){buff_point=0;}
			delay_point=buff_point-(del);if (delay_point<0){delay_point+=buffer_size;}
			sig_tot+=buffer[delay_point]/2;
			buffer[buff_point]=sig_tot;
			d_to_a((sig_tot/156072));
		}
		else {
			d_to_a((sig_tot/131073));
		}
	}
}

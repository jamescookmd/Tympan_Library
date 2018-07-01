
#ifndef _BTNRH_WDRC_TYPES_H
#define _BTNRH_WDRC_TYPES_H

namespace BTNRH_WDRC {

	// Here are the settings for the adaptive feedback cancelation
	typedef struct {
		int default_to_active; //enable AFC at startup?  1=active. 0=disabled.
		int afl;	//length (samples) of adaptive filter for modeling feedback path.
		float mu;	//mu, scale factor for how fast the adaptive filter adapts (bigger is faster)
		float rho;	//rho, smoothing factor for estimating audio envelope (bigger is a longer average)
		float eps;	//eps, when est the audio envelope, this is the min allowed level (avoids divide-by-zero)
	} CHA_AFC;

	// from CHAPRO cha_ff.h
	#define DSL_MXCH 32              
	//class CHA_DSL {
/* 	typedef struct {
		//public:
			//CHA_DSL(void) {};  
			//static const int DSL_MXCH = 32;    // maximum number of channels
			float attack;               // attack time (ms)
			float release;              // release time (ms)
			float maxdB;                // maximum signal (dB SPL)
			int ear;                     // 0=left, 1=right
			int nchannel;                // number of channels
			float cross_freq[DSL_MXCH]; // cross frequencies (Hz)
			float tkgain[DSL_MXCH];     // compression-start gain
			float cr[DSL_MXCH];         // compression ratio
			float tk[DSL_MXCH];         // compression-start kneepoint
			float bolt[DSL_MXCH];       // broadband output limiting threshold
	} CHA_DSL; */
	
	typedef struct {
	//public:
		//CHA_DSL(void) {};  
		//static const int DSL_MXCH = 32;    // maximum number of channels
		float attack;               // attack time (ms)
		float release;              // release time (ms)
		float maxdB;                // maximum signal (dB SPL)
		int ear;                     // 0=left, 1=right
		int nchannel;                // number of channels
		float cross_freq[DSL_MXCH]; // cross frequencies (Hz)
		float exp_cr[DSL_MXCH];		// compression ratio for low-SPL region (ie, the expander)
		float exp_end_knee[DSL_MXCH];	// expansion-end kneepoint
		float tkgain[DSL_MXCH];     // compression-start gain
		float cr[DSL_MXCH];         // compression ratio
		float tk[DSL_MXCH];         // compression-start kneepoint
		float bolt[DSL_MXCH];       // broadband output limiting threshold
	} CHA_DSL;
	
	/* 		int parseStringIntoDSL(String &text_buffer) {
			  int position = 0;
			  float foo_val;
			  const bool print_debug = false;
			
			  if (print_debug) Serial.println("parseTextAsDSL: values from file:");
			
			  position = parseNextNumberFromString(text_buffer, position, foo_val);
			  attack = foo_val;
			  if (print_debug) { Serial.print("  attack: "); Serial.println(attack); }
			
			  position = parseNextNumberFromString(text_buffer, position, foo_val);
			  release = foo_val;
			  if (print_debug) { Serial.print("  release: "); Serial.println(release); }
			
			  position = parseNextNumberFromString(text_buffer, position, foo_val);
			  maxdB = foo_val;
			  if (print_debug) { Serial.print("  maxdB: "); Serial.println(maxdB); }
			
			  position = parseNextNumberFromString(text_buffer, position, foo_val);
			  ear = int(foo_val + 0.5); //round
			  if (print_debug) { Serial.print("  ear: "); Serial.println(ear); }
			
			  position = parseNextNumberFromString(text_buffer, position, foo_val);
			  nchannel = int(foo_val + 0.5); //round
			  if (print_debug) { Serial.print("  nchannel: "); Serial.println(nchannel); }
			
			  //check to see if the number of channels is acceptable.
			  if ((nchannel < 0) || (nchannel > DSL_MXCH)) {
				if (print_debug) Serial.print("  : channel number is too big (or negative).  stopping."); 
				return -1;
			  }
			
			  //read the cross-over frequencies.  There should be nchan-1 of them (0 and Nyquist are assumed)
			  if (print_debug) Serial.print("  cross_freq: ");
			  for (int i=0; i < (nchannel-1); i++) {
				position = parseNextNumberFromString(text_buffer, position, foo_val);
				cross_freq[i] = foo_val;
				if (print_debug) { Serial.print(cross_freq[i]); Serial.print(", ");}
			  }
			  if (print_debug) Serial.println();
			
			  //read the tkgain values.  There should be nchan of them
			  if (print_debug) Serial.print("  tkgain: ");
			  for (int i=0; i < nchannel; i++) {
				position = parseNextNumberFromString(text_buffer, position, foo_val);
				tkgain[i] = foo_val;
				if (print_debug) { Serial.print(tkgain[i]); Serial.print(", ");}
			  }
			  if (print_debug) Serial.println();
			
			  //read the cr values.  There should be nchan of them
			  if (print_debug) Serial.print("  cr: ");
			  for (int i=0; i < nchannel; i++) {
				position = parseNextNumberFromString(text_buffer, position, foo_val);
				cr[i] = foo_val;
				if (print_debug) { Serial.print(cr[i]); Serial.print(", ");}
			  }
			  if (print_debug) Serial.println();
			
			  //read the tk values.  There should be nchan of them
			  if (print_debug) Serial.print("  tk: ");
			  for (int i=0; i < nchannel; i++) {
				position = parseNextNumberFromString(text_buffer, position, foo_val);
				tk[i] = foo_val;
				if (print_debug) { Serial.print(tk[i]); Serial.print(", ");}
			  }
			  if (print_debug) Serial.println();
			
			  //read the bolt values.  There should be nchan of them
			  if (print_debug) Serial.print("  bolt: ");
			  for (int i=0; i < nchannel; i++) {
				position = parseNextNumberFromString(text_buffer, position, foo_val);
				bolt[i] = foo_val;
				if (print_debug) { Serial.print(bolt[i]); Serial.print(", ");}
			  }
			  if (print_debug) Serial.println();
			
			  return 0;
			  
			}
			
			void printToStream(Stream *s) {
				s->print("CHA_DSL: attack (ms) = "); s->println(attack);
				s->print("    : release (ms) = "); s->println(release);
				s->print("    : maxdB (dB SPL) = "); s->println(maxdB);
				s->print("    : ear (0 = left, 1 = right) "); s->println(ear);
				s->print("    : nchannel = "); s->println(nchannel);
				s->print("    : cross_freq (Hz) = ");
					for (int i=0; i<nchannel-1;i++) { s->print(cross_freq[i]); s->print(", ");}; s->println();
				s->print("    : tkgain = ");
					for (int i=0; i<nchannel;i++) { s->print(tkgain[i]); s->print(", ");}; s->println();
				s->print("    : cr = ");
					for (int i=0; i<nchannel;i++) { s->print(cr[i]); s->print(", ");}; s->println();
				s->print("    : tk = ");
					for (int i=0; i<nchannel;i++) { s->print(tk[i]); s->print(", ");}; s->println();
				s->print("    : bolt = ");
					for (int i=0; i<nchannel;i++) { s->print(bolt[i]); s->print(", ");}; s->println();
			}
	} ; */
	
	typedef struct {
		float alfa;                 // attack constant (not time)
		float beta;                 // release constant (not time
		float fs;                   // sampling rate (Hz)
		float maxdB;                // maximum signal (dB SPL)
		float tkgain;               // compression-start gain
		float tk;                   // compression-start kneepoint
		float cr;                   // compression ratio
		float bolt;                 // broadband output limiting threshold
		//float gcalfa;				// ??  Added May 1, 2018 as part of adaptie feedback cancelation
		//float gcbeta;				// ??  Added May 1, 2018 as part of adaptie feedback cancelation
		float mu;				// 1e-3, adaptive feedback cancelation  (AFC) filter-estimation step size
		float rho;				// 0.984, AFC filter-estimation forgetting factor
		float pwr;				// estimated (smoothed) signal power for modifying mu for AFC
		float fbm;				// some sort of quality metric for AFC (some sort of power residual...smaller is better?)
		
	} CHA_DVAR_t;
	
	typedef struct {
		//int cs;  //chunk size (number of samples in data block)
		//int nc;  //number of frequency bands (channels)
		//int nw;	//number of samples in data window
		int rsz;    //32, ring buffer size (samples)?  Used for AFC
		int rhd;    //last starting point in the ring buffer (head)
		int rtl;    //last ending point in the ring buffer (tail)
		int afl;	//100, adaptive filter length for AFC
		int fbl;	//100, length of simulated feedback (ie, the given feedback impulse response)
		int nqm;    //number of quality metrics:  nqm = (fbl < afl) ? fbl : afl;
		
	} CHA_IVAR_t;
	
/* 	typedef struct {
		float attack;               // attack time (ms), unused in this class
		float release;              // release time (ms), unused in this class
		float fs;                   // sampling rate (Hz), set through other means in this class
		float maxdB;                // maximum signal (dB SPL)...I think this is the SPL corresponding to signal with rms of 1.0
		float tkgain;               // compression-start gain
		float tk;                   // compression-start kneepoint
		float cr;                   // compression ratio
		float bolt;                 // broadband output limiting threshold
	} CHA_WDRC; */
	
	typedef struct {
		float attack;               // attack time (ms), unused in this class
		float release;              // release time (ms), unused in this class
		float fs;                   // sampling rate (Hz), set through other means in this class
		float maxdB;                // maximum signal (dB SPL)...I think this is the SPL corresponding to signal with rms of 1.0
		float exp_cr;				// compression ratio for low-SPL region (ie, the expander)
		float exp_end_knee;			// expansion-end kneepoint
		float tkgain;               // compression-start gain
		float tk;                   // compression-start kneepoint
		float cr;                   // compression ratio
		float bolt;                 // broadband output limiting threshold
	} CHA_WDRC;
	

};

#endif


/*
 * AudioEffectCompBank_F32.cpp
 *
 * Chip Audette, OpenAudio, Aug 2021
 *
 * MIT License,  Use at your own risk.
 *
*/

#include <AudioEffectCompBankWDRC_F32.h>

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Compressor Bank State Methods
//
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// This method sets the value of the number of compressors being employed, as being tracked by this state-tracking class.
// This method doesn't change the algorithm's actual audio processing.
// To change the actual audio processing, do AudioEffectCompBankWDRC_F32::set_n_chan()
int AudioCompBankStateWDRC_F32::set_n_chan(int n) {
	if (compressors == NULL) return 0;
	if ((n < 0) || (n > max_n_chan)) return -1; //this is an error
	n_chan = n;
	return n_chan; 
}

/* //This method is purely internal to this state-tracking class.  It sets the size of the arrays used to hold
//the tracking of states for each compressor.  It must be equal to or larger than the actual number of channels
//tracked.
int AudioCompBankStateWDRC_F32::set_max_n_chan(int n) {
	//check inputs
	if ((n < 1) || (n > 64)) return -1; //-1 is an error
	
	//compare to current setting
	if (n <= max_n_chan) return 0;  //nothing to do, so return...zero is OK
	
	//we do need to expand our arrays, so first delete the current arrays
	//
	// <TO DO!!!!>   Example: if (crossover_freq_Hz != NULL) delete crossover_freq_Hz;  //delete arrays here
	//
	max_n_chan = 0; n_chan = 0;  //reset our tracking of the sizes
	
	//create new arrays
	//
	// <TO DO!!!!>   Example: crossover_freq_Hz = new float[n]; //create new arrays here
	//
	
	//return with error if memory allocation failed
	//
	// <TO DO!!!!>   Example: if (crossover_freq_Hz == NULL) return -1; //return if memory can't be allocated (-1 is an error)
	//
		
	//looks good!  let's return
	max_n_chan = n;
	return 0;  //zero is OK;
} */

//Tell this state-tracking class which compressors to reference when asking for the different
//state parameter values (so as we don't actually need to keep a local copy of each and every
//parameter of the compressors)
void AudioCompBankStateWDRC_F32::setCompressors(int n, AudioEffectCompWDRC_F32 c[]) {
	if (max_n_chan != n) {
		if (compressors != NULL) delete compressors;  //delete the pointer array that we allocated
		max_n_chan = n;
		compressors = new AudioEffectCompWDRC_F32*[max_n_chan];
	}

	for (int i=0; i<n; i++) compressors[i] = &(c[i]); //make an array of pointers to each instance of the class
	if (get_n_chan() > max_n_chan) set_n_chan(max_n_chan);
}


// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// 
// Compressor Bank Methods
//
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int AudioEffectCompBankWDRC_F32::configureFromDSLandGHA(float fs_Hz, const BTNRH_WDRC::CHA_DSL &this_dsl, const BTNRH_WDRC::CHA_WDRC &this_gha) {
	
	//sorta check the validity of inputs
	int n_chan_to_load = this_dsl.nchannel;
	int max_n_chan = state.get_max_n_chan();
	if (n_chan_to_load > max_n_chan) {
		Serial.println(F("AudioEffectCompBankWDRC_F32: configureFromDSLandGHA: *** ERROR ***: dsl.nchannel > max_n_chan"));
		Serial.print(F("    : dsl.nchannel = ")); Serial.println(this_dsl.nchannel);
		Serial.print(F("    : max_n_chan = ")); Serial.println(max_n_chan);
		Serial.println(F("    : Loading only up to max_n_chan.  Continuing..."));
		n_chan_to_load = max_n_chan;
	} 
	
	//parse the inputs that apply to all channels [logic and values are extracted from from CHAPRO repo agc_prepare.c]
	float atk = (float)this_dsl.attack;   //milliseconds!
	float rel = (float)this_dsl.release;  //milliseconds!
	float fs = (float)fs_Hz; // override based on the global setting...was float fs = gha->fs;
	float maxdB = (float) this_dsl.maxdB;
	
	//now, loop over each channel
	for (int i=0; i < n_chan_to_load; i++) {

		//parse the per-channel inputs
		float exp_cr = (float)this_dsl.exp_cr[i];
		float exp_end_knee = (float)this_dsl.exp_end_knee[i];
		float tk = (float) this_dsl.tk[i];
		float comp_ratio = (float) this_dsl.cr[i];
		float tkgain = (float) this_dsl.tkgain[i];
		float bolt = (float) this_dsl.bolt[i];

		// adjust BOLT
		float cltk = (float)this_gha.tk;
		if (bolt > cltk) bolt = cltk;
		if (tkgain < 0) bolt = bolt + tkgain;

		//set the compressor's parameters
		compressors[i].setSampleRate_Hz(fs);
		compressors[i].setParams(atk,rel,maxdB,exp_cr,exp_end_knee,tkgain,comp_ratio,tk,bolt);
	}
		
	return n_chan_to_load;  //returns number of channels loaded from the DSL
	
}

void AudioEffectCompBankWDRC_F32::update(void) {
	//return if not enabled
	if (!is_enabled) return;
	
	//loop over each channel...but only those up to the active channel limit
	int n_chan = state.get_n_chan();
	for (int Ichan=0; Ichan < n_chan; Ichan++) {
		
		 //request the in-coming data block
		audio_block_f32_t *block = AudioStream_F32::receiveReadOnly_f32(Ichan);
		
		if (block) { //did we get a block of data?
		
			//request a data block to hold th processed data
			audio_block_f32_t *out_block = AudioStream_F32::allocate_f32();
			
			if (out_block) { //did we get a valid memory handle?
				//do the algorithm
				int is_error = compressors[Ichan].processAudioBlock(block,out_block); //anything other than a zero is an error
				
				//if we had no error, transmit the processed data
				if (!is_error) AudioStream_F32::transmit(out_block, Ichan);
			}
			
			AudioStream_F32::release(out_block);  //release the memory block that we requested 
		}
		
		AudioStream_F32::release(block); //release the memory block that we requested
	} 
}

int AudioEffectCompBankWDRC_F32::set_n_chan(int val) {
	val = min(val, __MAX_NUM_COMP); 
	int n_chan = state.set_n_chan(val);
	for (int Ichan = 0; Ichan < __MAX_NUM_COMP; Ichan++) {
		if (Ichan < n_chan) {
			//compressors[Ichan].enable(true);  //enable the individual compressor [there is no such method?]
		} else {
			//compressors[Ichan].enable(false); //disable the individual compressor [there is no such method?]
		}
	}		
	return n_chan;
}

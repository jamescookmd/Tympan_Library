/*
*   Process audio by applying a high-pass filter followed by gain.
*   Blue potentiometer adjusts the digital gain applied to the filtered audio signal.
*/

#include <Tympan_Library.h>

#include <AudioStream_F32.h>  // for AudioConnection_F32

const float kSampleRateHz = 24000.0f ; //24000 or 44117 (or other frequencies in the table in AudioOutputI2S_F32)
const int kAudioBlockSamples = 32;     //do not make bigger than AUDIO_BLOCK_SAMPLES from AudioStream.h (which is 128)
AudioSettings_F32 audio_settings(kSampleRateHz, kAudioBlockSamples);

// Create audio library objects for handling the audio.
Tympan g_tympan(TympanRev::E, audio_settings);                // TympanRev::D or TympanRev::E
AudioInputI2S_F32 i2s_in(audio_settings);                     // Digital audio *from* the Tympan AIC.
AudioFilterBiquad_F32 left_high_pass_filter(audio_settings);  // Biquad (IIR) highpass filter.
AudioFilterBiquad_F32 right_high_pass_filter(audio_settings); // Biquad (IIR) highpass filter.
AudioEffectGain_F32 left_gain;                                // Digital gain.
AudioEffectGain_F32 right_gain;                               // Digital gain.
AudioOutputI2S_F32 i2s_out(audio_settings);                   // Digital audio *to* the Tympan AIC.  Always list last to minimize latency

// Make all of the audio connections.
const unsigned char kMono = 0;
const unsigned char kLeft = 0;
const unsigned char kRight = 1;
AudioConnection_F32 patch_cord1(i2s_in, kLeft, left_high_pass_filter, kMono);      // Left input
AudioConnection_F32 patch_cord2(i2s_in, kRight, right_high_pass_filter, kMono);    // Right input
AudioConnection_F32 patch_cord3(left_high_pass_filter, kMono, left_gain, kMono);   // Left
AudioConnection_F32 patch_cord4(right_high_pass_filter, kMono, right_gain, kMono); // Right
AudioConnection_F32 patch_cord5(left_gain, kMono, i2s_out, kLeft);                 // Left gain to the Left output
AudioConnection_F32 patch_cord6(right_gain, kMono, i2s_out, kRight);               // Right gain to the Right output

// setup() is called once when the device is booting.
const float kInputGainDb = 20.0f; //gain on the microphone
// float vol_knob_gain_dB = 0.0;      //will be overridden by volume knob
void setup() {
  // Begin the serial comms (for debugging). use the print functions in "g_tympan" so it goes to BT, too!
  g_tympan.beginBothSerial();
  delay(1000);
  g_tympan.println("TrebleBoost: Starting setup()...");

  // Allocate the dynamic memory for audio processing blocks
  AudioMemory_F32(10,audio_settings); 

  // Enable the Tympan to start the audio flowing!
  g_tympan.enable(); // activate AIC

  // Choose the desired input.
  g_tympan.inputSelect(TYMPAN_INPUT_ON_BOARD_MIC);     // on board microphones
  //g_tympan.inputSelect(TYMPAN_INPUT_JACK_AS_MIC);    // microphone jack - defaults to mic bias 2.5V
  //g_tympan.inputSelect(TYMPAN_INPUT_JACK_AS_LINEIN); // microphone jack - defaults to mic bias OFF

  // Set the desired volume levels.
  g_tympan.volume_dB(0.f);                 // headphone amplifier.  -63.6 to +24 dB in 0.5dB steps.
  g_tympan.setInputGain_dB(kInputGainDb);  // set input volume, 0-47.5dB in 0.5dB setps

  // Set the cutoff frequency for the high pass filter. Frequencies below this
  // will be attenuated.
  const float kCutoffHz = 1000.f;  //frequencies below this will be attenuated
  g_tympan.print("Highpass filter cutoff at ");
  g_tympan.print(kCutoffHz);
  g_tympan.println(" Hz");
  const uint32_t kStage = 0;
  left_high_pass_filter.setHighpass(kStage, kCutoffHz); //biquad IIR filter.  left channel
  right_high_pass_filter.setHighpass(kStage, kCutoffHz); //biquad IIR filter.  right channel

  // Check the volume knob.
  servicePotentiometer(millis(),0);  //the "0" is not relevant here.

  g_tympan.println("Setup complete.");
} //end setup()


// loop() is repeated over and over for the life of the device.
void loop() {
  // Service the potentiometer every 100 ms.
  servicePotentiometer(millis(),100);

  // Print the CPU and Memory Usage every 3000 ms.
  g_tympan.printCPUandMemory(millis(),3000);

  // Blink the LEDs!
  g_tympan.serviceLEDs(millis());   //defaults to a slow toggle (see Tympan.h and Tympan.cpp)
}

// servicePotentiometer() listens to the blue potentiometer and sends the new pot value
// to the audio processing algorithm as a control parameter
// `update_period_ms` is the delay between updates
void servicePotentiometer(unsigned long cur_time_ms, unsigned long update_period_ms) {
  static unsigned long last_update_ms = 0;
  static float prev_val = -1.0f;

  // Has enough time passed to update everything?
  if (cur_time_ms < last_update_ms) last_update_ms = 0; // handle wrap-around of the clock
  if ((cur_time_ms - last_update_ms) <= update_period_ms) {
    // Not yet time to update.
    return;
  }

  // Read potentiometer and quantize 0.0 to 1.0.
  const float raw_val = float(g_tympan.readPotentiometer()) / 1023.0f; //0.0 to 1.0
  const float val = (1.f / 9.f) * (float)((int)(9.f * raw_val + 0.5f));

  // Check for changes.
  if (abs(val - prev_val) <= 0.05)
    return;

  // Save the value for comparison for the next time around
  prev_val = val;

  // Choose the desired gain value based on the knob setting
  const float kMinGain = -20.0;  // dB
  const float kMaxGain = 40.0;   // dB
  const float vol_knob_gain_db = kMinGain + (kMaxGain - kMinGain) * val;

  //command the new gain setting
  left_gain.setGain_dB(vol_knob_gain_db);  //set the gain of the Left-channel gain processor
  right_gain.setGain_dB(vol_knob_gain_db);  //set the gain of the Right-channel gain processor
  g_tympan.print("servicePotentiometer: Digital Gain dB = "); g_tympan.println(vol_knob_gain_db); //print text to Serial port for debugging
  last_update_ms = cur_time_ms;
} //end servicePotentiometer();

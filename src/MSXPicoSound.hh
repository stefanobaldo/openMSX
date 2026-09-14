#ifndef MSXPICOSOUND_HH
#define MSXPICOSOUND_HH

#include "ResampledSoundDevice.hh"

#include <cstdint>
#include <span>
#include <vector>

namespace openmsx {

class DeviceConfig;
class MSXPicoBridge;

/** The cartridge's I2S output as a sound device: one stereo channel carrying
  * the mix the firmware's own PSG, SCC and FM emulators made, at the rate the
  * firmware programmed on its I2S clock.
  *
  * The samples are pulled, not pushed. On the chip the I2S interrupt is the
  * audio's clock and the firmware's handler fills the FIFO one word per
  * interrupt; here the mixer asks for the samples emulated time has produced,
  * and the library runs the same handler that many times. So the audio shares
  * the Z80's clock: a paused machine is silent, a machine at 25x is 25x, and a
  * recording is the same on every run.
  *
  * The firmware retunes its I2S clock to a WAV or MP3 file's rate while it
  * plays one; the device follows by recreating its resampler. Between one
  * life of the firmware and the next (a reboot) the device is silent.
  */
class MSXPicoSound final : public ResampledSoundDevice
{
public:
	MSXPicoSound(MSXPicoBridge& bridge, const DeviceConfig& config);
	~MSXPicoSound();

	bool updateBuffer(size_t length, float* buffer, EmuTime time) override;

private:
	void generateChannels(std::span<float*> bufs, unsigned num) override;

	MSXPicoBridge& bridge;
	std::vector<int16_t> frames; // scratch: 2 * num
};

} // namespace openmsx

#endif

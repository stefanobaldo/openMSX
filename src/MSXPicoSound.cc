#include "MSXPicoSound.hh"

#include "MSXPicoBridge.hh"

#include "DeviceConfig.hh"
#include "MSXException.hh"

#include <cassert>

namespace openmsx {

MSXPicoSound::MSXPicoSound(MSXPicoBridge& bridge_, const DeviceConfig& config)
	: ResampledSoundDevice(config.getMotherBoard(), "MSX-Pico",
	                       "MSX-Pico+ cartridge: the firmware's own sound",
	                       1, bridge_.sampleRate(), true)
	, bridge(bridge_)
{
	// registerSound() calls setOutputRate() at once, which builds a resampler
	// over the input rate; the firmware has programmed its I2S clock before
	// msxpico_init returns, so a zero here is a broken library, not a race.
	if (getInputRate() == 0) {
		throw MSXException("MSXPicoBridge: the firmware reported no sample rate");
	}
	registerSound(config);
}

MSXPicoSound::~MSXPicoSound()
{
	unregisterSound();
}

bool MSXPicoSound::updateBuffer(size_t length, float* buffer, EmuTime time)
{
	// The firmware retunes its I2S clock to a WAV or MP3 file's rate and back
	// (sound_set_sample_freq). Follow it here, outside the resampler's own
	// call -- generateChannels() runs inside it, and must not replace it.
	if (uint32_t rate = bridge.sampleRate(); rate != 0 && rate != getInputRate()) {
		setInputRate(rate);
		createResampler();
	}
	return ResampledSoundDevice::updateBuffer(length, buffer, time);
}

void MSXPicoSound::generateChannels(std::span<float*> bufs, unsigned num)
{
	// One stereo channel: 2 * num floats, left/right interleaved. A single-
	// channel device replaces the buffer's content rather than adding to it.
	assert(bufs.size() == 1);
	frames.resize(size_t(num) * 2);
	size_t got = bridge.pullSamples(frames.data(), num);
	if (got == 0) {
		bufs[0] = nullptr; // no live firmware: silence, the cheap way
		return;
	}
	float* out = bufs[0];
	// The ABI's int16 is the range SoundDevice's default amplification factor
	// (1 / 32768) expects; no scaling here.
	for (size_t i = 0; i < got * 2; ++i) out[i] = float(frames[i]);
	for (size_t i = got * 2; i < size_t(num) * 2; ++i) out[i] = 0.0f;
}

} // namespace openmsx

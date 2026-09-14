#ifndef MSXPICOBRIDGE_HH
#define MSXPICOBRIDGE_HH

#include "MSXDevice.hh"

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace openmsx {

class MSXPicoInstance;
class MSXPicoSound;

/** A cartridge whose firmware runs as a host library implementing the
  * libmsxpico C ABI (msxpico.h). The library is configured by children of the
  * device element: <library> (the shared library) and <flash> (the flash image
  * it reads and writes) are required, <library2> is optional, and all are
  * resolved through the extension's file context. The device claims the whole
  * slot and all I/O ports; the firmware decides what it answers.
  *
  * One MSXPicoInstance per life of the firmware: a REBOOT event ends a life
  * and starts the next; a HALTED event ends a life for good, until the MSX is
  * power-cycled.
  *
  * Which library starts the next life is the cartridge's own choice, taken the
  * way its bootloader takes it. The MSX-Pico+ ships two firmware images that
  * differ only in whether the FM synthesiser is built in, and after every
  * reset its msc_bootloader jumps to one or the other according to watchdog
  * scratch register 3 — which the menu writes when <INS> toggles FM, and the
  * configuration screen writes when that setting is saved. The REBOOT event
  * carries those registers, so <library> is firmware 1 (no FM) and <library2>
  * is firmware 2 (FM); a reboot asking for firmware 2 with no <library2>
  * configured reloads firmware 1 and says so once.
  *
  * Those registers are hardware on the chip and survive the reset the firmware
  * triggers, which is the whole reason it uses them; a library image does not
  * survive it, so this class holds them and hands them to the next image
  * through msxpico_config. Without that the firmware forgets across every
  * reboot, and the FM toggle switches image for exactly one reboot before the
  * MSX's own reset undoes it.
  *
  * There is no bootloader here to choose the *first* library, so the XML says:
  * the optional child <fm> starts on <library2>. It is this host's stand-in for
  * the saved setting the bootloader reads, and a power cycle returns to it
  * rather than to whatever <INS> last asked for.
  *
  * <sd> is optional and names the SD card's image. It is resolved like
  * <flash>, handed to the library as sd_image_path, and reopened by every
  * life, so what one life writes the next one reads — a card that stays in the
  * slot across a reset.
 *
  * <sound> is required, as for every openMSX sound device: the bridge
  * registers one stereo sound device, MSX-Pico, whose samples are the
  * firmware's I2S output pulled from the library when the mixer asks (see
  * MSXPicoSound).
  */
class MSXPicoBridge final : public MSXDevice
{
public:
	explicit MSXPicoBridge(const DeviceConfig& config);
	~MSXPicoBridge() override;

	void reset(EmuTime time) override;
	void powerUp(EmuTime time) override;

	[[nodiscard]] byte readMem(uint16_t address, EmuTime time) override;
	void writeMem(uint16_t address, byte value, EmuTime time) override;
	[[nodiscard]] byte readIO(uint16_t port, EmuTime time) override;
	void writeIO(uint16_t port, byte value, EmuTime time) override;

	/** For MSXPicoSound: the firmware's current I2S sample rate in Hz, or 0
	  * when no life is live. */
	[[nodiscard]] uint32_t sampleRate() const;
	/** For MSXPicoSound: `frames` stereo frames of the firmware's output into
	  * `out`, interleaved left/right; returns 0 when no life is live. */
	size_t pullSamples(int16_t* out, size_t frames);

private:
	void load();       // starts a life from `scratch`; throws MSXException
	void coldScratch(); // the registers a cold start begins with
	void pollEvents(); // drains the log queue, then handles one ABI event

	std::string libraryPath;   // firmware 1, without the FM synthesiser
	std::string library2Path;  // firmware 2, with it; empty when not configured
	std::string flashPath;
	std::string sdPath;      // the card's image; empty when no <sd>
	bool fmAtPowerUp = false;  // the <fm> element: which library a power-up starts
	bool warnedNoLibrary2 = false;
	// The cartridge's watchdog scratch registers, which outlive a library
	// image the way the chip's registers outlive a reset (msxpico.h).
	std::array<uint32_t, 4> scratch = {};
	std::unique_ptr<MSXPicoInstance> instance;
	bool live = false; // false: the bus answers 0xFF without calling the library
	// After `instance` and `live`, so it is destroyed first: it unregisters
	// from the mixer while the instance it pulls from is still there.
	std::unique_ptr<MSXPicoSound> sound;
};

} // namespace openmsx

#endif

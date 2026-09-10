#ifndef MSXPICOBRIDGE_HH
#define MSXPICOBRIDGE_HH

#include "MSXDevice.hh"

#include <memory>
#include <string>

namespace openmsx {

class MSXPicoInstance;

/** A cartridge whose firmware runs as a host library implementing the
  * libmsxpico C ABI (msxpico.h). The library is configured by two children of
  * the device element: <library> (the shared library) and <flash> (the flash
  * image it reads and writes), both resolved through the extension's file
  * context. The device claims the whole slot and all I/O ports; the firmware
  * decides what it answers.
  *
  * One MSXPicoInstance per life of the firmware: a REBOOT event ends a life
  * and starts the next; a HALTED event ends a life for good, until the MSX is
  * power-cycled.
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

private:
	void load();       // starts a life; throws MSXException
	void pollEvents(); // drains the log queue, then handles one ABI event

	std::string libraryPath;
	std::string flashPath;
	std::unique_ptr<MSXPicoInstance> instance;
	bool live = false; // false: the bus answers 0xFF without calling the library
};

} // namespace openmsx

#endif

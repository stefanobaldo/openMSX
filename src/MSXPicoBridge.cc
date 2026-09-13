#include "MSXPicoBridge.hh"

#include "DeviceConfig.hh"
#include "FileContext.hh"
#include "FileException.hh"
#include "FileOperations.hh"
#include "MSXCliComm.hh"
#include "MSXException.hh"
#include "XMLElement.hh"
#include "narrow.hh"
#include "strCat.hh"

#include "msxpico.h"

#include <dlfcn.h>

#include <algorithm>
#include <array>
#include <cstdio>
#include <deque>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>

namespace openmsx {

// Log levels of the library's callback, named by the ABI since version 2.
// Anything above INFO is treated as a warning.
static constexpr int LOG_DEBUG = MSXPICO_LOG_DEBUG;
static constexpr int LOG_INFO  = MSXPICO_LOG_INFO;

static const char* errorName(int rc)
{
	switch (rc) {
	case MSXPICO_ERR_CONFIG: return "MSXPICO_ERR_CONFIG";
	case MSXPICO_ERR_FLASH:  return "MSXPICO_ERR_FLASH";
	case MSXPICO_ERR_HALTED: return "MSXPICO_ERR_HALTED";
	case MSXPICO_ERR_STALE:  return "MSXPICO_ERR_STALE";
	default:                 return "unknown error";
	}
}

/** One life of the firmware: a private copy of the library, dlopen'ed,
  * initialised, and torn down in the destructor. macOS does not reliably
  * unload images, so a new life is always a new copy. */
class MSXPicoInstance
{
public:
	MSXPicoInstance(const std::string& libraryPath, const std::string& flashPath,
	                const std::string& sdPath,
	                const std::array<uint32_t, 4>& scratch);
	~MSXPicoInstance();
	MSXPicoInstance(const MSXPicoInstance&) = delete;
	MSXPicoInstance& operator=(const MSXPicoInstance&) = delete;

	[[nodiscard]] int read(uint16_t addr) const { return api.read(addr); }
	void write(uint16_t addr, uint8_t data) const { api.write(addr, data); }
	[[nodiscard]] int readIO(uint16_t port) const { return api.readIO(port); }
	void writeIO(uint16_t port, uint8_t data) const { api.writeIO(port, data); }
	[[nodiscard]] bool pollEvent(msxpico_event& out) const {
		return api.pollEvent(&out) == 1;
	}

	/** Removes and returns the queued log lines, oldest first. Called on the
	  * emulator thread only. */
	[[nodiscard]] std::deque<std::pair<int, std::string>> takeLogs();
	/** The most recent WARN-or-worse line; failing that, the most recent line
	  * of any level. */
	[[nodiscard]] std::string lastError() const;

private:
	static void logCallback(int level, const char* msg, void* user);
	void copyLibrary(const std::string& libraryPath);
	void teardown();

	std::string copyPath;
	void* handle = nullptr;
	struct {
		uint32_t (*abiVersion)() = nullptr;
		int (*init)(const msxpico_config*) = nullptr;
		void (*shutdown)() = nullptr;
		int (*read)(uint16_t) = nullptr;
		void (*write)(uint16_t, uint8_t) = nullptr;
		int (*readIO)(uint16_t) = nullptr;
		void (*writeIO)(uint16_t, uint8_t) = nullptr;
		int (*pollEvent)(msxpico_event*) = nullptr;
		size_t (*pullSamples)(int16_t*, size_t) = nullptr;
	} api;
	bool initCalled = false;

	mutable std::mutex mutex;
	std::deque<std::pair<int, std::string>> logs;
	std::string lastWarn;
	std::string lastAny;
};

MSXPicoInstance::MSXPicoInstance(const std::string& libraryPath, const std::string& flashPath,
                                 const std::string& sdPath,
                                 const std::array<uint32_t, 4>& scratch)
{
	try {
		copyLibrary(libraryPath);
		handle = dlopen(copyPath.c_str(), RTLD_NOW | RTLD_LOCAL);
		if (!handle) {
			throw MSXException("cannot load ", libraryPath, ": ", dlerror());
		}
		auto resolve = [&](auto& fn, const char* name) {
			fn = reinterpret_cast<std::remove_reference_t<decltype(fn)>>(dlsym(handle, name));
			if (!fn) {
				throw MSXException(libraryPath, " does not export ", name);
			}
		};
		resolve(api.abiVersion,  "msxpico_abi_version");
		resolve(api.init,        "msxpico_init");
		resolve(api.shutdown,    "msxpico_shutdown");
		resolve(api.read,        "msxpico_read");
		resolve(api.write,       "msxpico_write");
		resolve(api.readIO,      "msxpico_read_io");
		resolve(api.writeIO,     "msxpico_write_io");
		resolve(api.pollEvent,   "msxpico_poll_event");
		resolve(api.pullSamples, "msxpico_pull_samples");

		if (auto v = api.abiVersion(); v != MSXPICO_ABI_VERSION) {
			throw MSXException(libraryPath, " implements ABI version ", v,
			                   ", this openMSX expects ", MSXPICO_ABI_VERSION);
		}

		msxpico_config cfg{};
		cfg.struct_size = sizeof(msxpico_config);
		cfg.flash_image_path = flashPath.c_str();
		cfg.sd_image_path = sdPath.empty() ? nullptr : sdPath.c_str();
		cfg.log = &MSXPicoInstance::logCallback;
		cfg.log_user = this;
		// What the chip's watchdog registers would still hold: the previous
		// life's values after a reset, zeroes after losing power.
		std::copy(scratch.begin(), scratch.end(), cfg.scratch);
		initCalled = true;
		if (int rc = api.init(&cfg); rc != MSXPICO_OK) {
			throw MSXException("msxpico_init failed: ", errorName(rc), " (", lastError(), ')');
		}
	} catch (...) {
		teardown();
		throw;
	}
}

MSXPicoInstance::~MSXPicoInstance()
{
	teardown();
}

void MSXPicoInstance::copyLibrary(const std::string& libraryPath)
{
	auto in = FileOperations::openFile(libraryPath, "rb");
	if (!in) {
		throw MSXException("cannot open ", libraryPath);
	}
	auto out = FileOperations::openUniqueFile(FileOperations::getTempDir(), copyPath);
	if (!out) {
		throw MSXException("cannot create a private copy of ", libraryPath);
	}
	std::array<char, 65536> buf;
	while (size_t n = fread(buf.data(), 1, buf.size(), in.get())) {
		if (fwrite(buf.data(), 1, n, out.get()) != n) {
			throw MSXException("cannot write ", copyPath);
		}
	}
}

void MSXPicoInstance::teardown()
{
	if (initCalled && api.shutdown) {
		api.shutdown();
		initCalled = false;
	}
	if (handle) {
		dlclose(handle);
		handle = nullptr;
	}
	if (!copyPath.empty()) {
		FileOperations::unlink(copyPath);
		copyPath.clear();
	}
}

void MSXPicoInstance::logCallback(int level, const char* msg, void* user)
{
	// Called from the library's own threads: only queue, never print.
	auto& self = *static_cast<MSXPicoInstance*>(user);
	std::scoped_lock lock(self.mutex);
	self.logs.emplace_back(level, msg ? msg : "");
	self.lastAny = self.logs.back().second;
	if (level > LOG_INFO) self.lastWarn = self.lastAny;
}

std::deque<std::pair<int, std::string>> MSXPicoInstance::takeLogs()
{
	std::scoped_lock lock(mutex);
	return std::exchange(logs, {});
}

std::string MSXPicoInstance::lastError() const
{
	std::scoped_lock lock(mutex);
	return lastWarn.empty() ? lastAny : lastWarn;
}


MSXPicoBridge::MSXPicoBridge(const DeviceConfig& config)
	: MSXDevice(config)
{
	const auto& xml = getDeviceConfig();
	const auto* lib = xml.findChild("library");
	if (!lib) throw MSXException("MSXPicoBridge: <library> is required");
	const auto* flash = xml.findChild("flash");
	if (!flash) throw MSXException("MSXPicoBridge: <flash> is required");
	const auto& context = config.getFileContext();
	try {
		libraryPath = context.resolve(lib->getData());
	} catch (FileException& e) {
		throw MSXException("MSXPicoBridge: library \"", lib->getData(),
		                   "\" not found: ", e.getMessage());
	}
	try {
		flashPath = context.resolve(flash->getData());
	} catch (FileException& e) {
		throw MSXException("MSXPicoBridge: flash image \"", flash->getData(),
		                   "\" not found: ", e.getMessage());
	}
	// Optional: the firmware the cartridge's bootloader would jump to when its
	// scratch register asks for FM. Without it the bridge has one firmware and
	// behaves as it always did.
	if (const auto* lib2 = xml.findChild("library2")) {
		try {
			library2Path = context.resolve(lib2->getData());
		} catch (FileException& e) {
			throw MSXException("MSXPicoBridge: library2 \"", lib2->getData(),
			                   "\" not found: ", e.getMessage());
		}
	}
	// Optional: the SD card's image. Resolved like <flash>; the library opens
	// it read/write and every life reopens the same file.
	if (const auto* sd = xml.findChild("sd")) {
		try {
			sdPath = context.resolve(sd->getData());
		} catch (FileException& e) {
			throw MSXException("MSXPicoBridge: sd image \"", sd->getData(),
			                   "\" not found: ", e.getMessage());
		}
	}
	fmAtPowerUp = xml.getChildDataAsBool("fm", false);
	if (fmAtPowerUp && library2Path.empty()) {
		throw MSXException("MSXPicoBridge: <fm> is set but <library2> is missing");
	}
	coldScratch();
	load();
}

void MSXPicoBridge::coldScratch()
{
	// A chip that has just been given power: every register clear, except the
	// one the bootloader fills in from the saved setting before it jumps.
	scratch = {};
	scratch[3] = fmAtPowerUp ? 1u : 0u;
}

MSXPicoBridge::~MSXPicoBridge() = default;

void MSXPicoBridge::load()
{
	// Scratch register 3 is what the cartridge's bootloader reads to choose
	// the firmware image, so it is what chooses here.
	bool fm = scratch[3] != 0;
	// Firmware 2 only if it is configured. Asking for a firmware that is not
	// there is worth saying once -- the menu will come back with FM still off,
	// which looks like the toggle having done nothing -- but it is not worth
	// killing the cartridge over.
	if (fm && library2Path.empty() && !warnedNoLibrary2) {
		warnedNoLibrary2 = true;
		getCliComm().printWarning(
			"MSX-Pico: the firmware asked for the FM build, but no <library2> "
			"is configured; reloading the one there is, with FM off");
	}
	const std::string& path = (fm && !library2Path.empty()) ? library2Path : libraryPath;
	try {
		instance = std::make_unique<MSXPicoInstance>(path, flashPath, sdPath, scratch);
	} catch (MSXException& e) {
		throw MSXException("MSXPicoBridge: ", e.getMessage());
	}
	live = true;
	pollEvents();
}

void MSXPicoBridge::pollEvents()
{
	if (!instance) return;
	for (const auto& [level, text] : instance->takeLogs()) {
		if (level == LOG_DEBUG) continue;
		if (level == LOG_INFO) {
			getCliComm().printInfo(strCat("MSX-Pico: ", text));
		} else {
			getCliComm().printWarning(strCat("MSX-Pico: ", text));
		}
	}
	if (!live) return;
	msxpico_event ev{};
	if (!instance->pollEvent(ev)) return;
	if (ev.kind == MSXPICO_EVENT_REBOOT) {
		// The firmware rebooted (watchdog_reboot): a new life, right now,
		// inside the bus cycle that caused it, as the cartridge does. The
		// registers it leaves behind survive the reset, so they cross over to
		// the next life: register 3 is what the bootloader reads to pick the
		// firmware image, written by <INS> in the menu and by saving the FM
		// setting from the configuration screen.
		std::copy(std::begin(ev.scratch), std::end(ev.scratch), scratch.begin());
		instance.reset();
		live = false;
		try {
			load();
		} catch (MSXException& e) {
			getCliComm().printWarning(strCat(
				"MSX-Pico: the firmware rebooted but could not be reloaded: ",
				e.getMessage()));
		}
	} else if (ev.kind == MSXPICO_EVENT_HALTED) {
		live = false;
		getCliComm().printWarning(strCat(
			"MSX-Pico: firmware halted: ", instance->lastError(),
			" (power the MSX off and on to restart it)"));
	}
}

void MSXPicoBridge::reset(EmuTime /*time*/)
{
	// Nothing: the firmware detects the MSX reset itself, from the PPI writes
	// it sees on the bus, exactly as the cartridge does.
}

void MSXPicoBridge::powerUp(EmuTime time)
{
	MSXDevice::powerUp(time);
	if (live) return;
	// A power cycle is what unsticks a stuck cartridge, and it is also what
	// clears the watchdog registers: the next life starts from the saved
	// setting, not from whatever <INS> last asked for.
	instance.reset();
	coldScratch();
	try {
		load();
	} catch (MSXException& e) {
		getCliComm().printWarning(strCat(
			"MSX-Pico: the firmware could not be restarted: ", e.getMessage()));
	}
}

byte MSXPicoBridge::readMem(uint16_t address, EmuTime /*time*/)
{
	if (!live) return 0xFF;
	int r = instance->read(address);
	pollEvents();
	return (r < 0) ? byte(0xFF) : narrow_cast<byte>(r);
}

void MSXPicoBridge::writeMem(uint16_t address, byte value, EmuTime /*time*/)
{
	if (!live) return;
	instance->write(address, value);
	pollEvents();
}

byte MSXPicoBridge::readIO(uint16_t port, EmuTime /*time*/)
{
	if (!live) return 0xFF;
	int r = instance->readIO(port); // the full 16-bit port, B included
	pollEvents();
	return (r < 0) ? byte(0xFF) : narrow_cast<byte>(r);
}

void MSXPicoBridge::writeIO(uint16_t port, byte value, EmuTime /*time*/)
{
	if (!live) return;
	instance->writeIO(port, value);
	pollEvents();
}

REGISTER_MSXDEVICE(MSXPicoBridge, "MSXPicoBridge");

} // namespace openmsx

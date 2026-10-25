// Opt-in MCP25xxFD foreground engine. Legacy ACAN2517FD entry points are unchanged.
// One core owns every method. The interrupt handler must never call this engine.
#pragma once
#include <stddef.h>
#include <stdint.h>

// Only the opt-in foreground path uses SRAM code placement. Native tests keep
// ordinary functions; legacy ACAN entry points and their section layout are untouched.
#if defined(ARDUINO_ARCH_RP2040) && defined(__GNUC__)
#define ACAN_FOREGROUND_RAM(name) __attribute__((section(".time_critical.dt3_can_" #name), noinline))
#else
#define ACAN_FOREGROUND_RAM(name)
#endif

namespace acan2517fd {

struct ForegroundFrame {
    uint32_t id = 0;
    uint8_t length = 0;
    uint8_t flags = 3; // bit 0: FD, bit 1: BRS, bit 2: extended, bit 3: RTR
    uint16_t reserved = 0;
    uint8_t data[64]{};
    uint64_t queuedUs = 0; // owner-local MCU clock, never a physical bus timestamp
};

// Separate register IO permits independent SPI/register-model tests. read/write
// complete one bounded transaction; reset is the two-byte RESET instruction.
class ForegroundIo {
  public:
    virtual ~ForegroundIo() = default;
    virtual bool read(uint16_t address, uint8_t *bytes, size_t count) = 0;
    virtual bool write(uint16_t address, const uint8_t *bytes, size_t count) = 0;
    virtual bool reset() = 0;
    virtual void speed(uint32_t hz) = 0;
    virtual uint64_t timestampUs() const = 0;
};

struct ForegroundConfig {
    uint8_t hardwareRx = 24, hardwareTx = 4;
    uint8_t softwareRx = 32, softwareTx = 4;
    bool loopback = false;
    uint16_t requestId = 0x6E0;
};

enum class ForegroundState : uint8_t { Off, Initializing, Ready, Recovering, Faulted };
enum class ForegroundFault : uint8_t {
    None,
    Spi,
    ModeTimeout,
    InitializationTimeout,
    BusOff,
    AttemptsExhausted,
    System,
    SpiCrc,
    Ecc,
    PersistentInterrupt,
    Configuration
};
enum class ForegroundWork : uint8_t { None, Initialization, Receive, Transmit, Maintenance, Recovery };
// Diagnostic IDs are stable across the bench unit-timing extension. One ready
// state unit performs one SPI transaction or one queue-bookkeeping operation.
enum class ForegroundUnit : uint8_t {
    None, VectorStatus, RxAddress, RxObject, RxAdvance, RxPublish,
    TxStatusAddress, TxPrepare, TxObject, TxAdvance, TxComplete,
    Health, FaultClear, Initialization, Recovery
};

struct ForegroundCounters {
    uint32_t rxFrames = 0, rxBytes = 0, txFrames = 0, txBytes = 0;
    uint32_t rxHigh = 0, txHigh = 0, hardwareOverflow = 0, softwareOverflow = 0;
    uint32_t rxWindowHigh = 0, txWindowHigh = 0;
    uint32_t txRejected = 0, attemptsExhausted = 0, busOff = 0;
    uint32_t recoveryAttempts = 0, recoveries = 0, faults = 0;
    uint32_t trec = 0, vector = 0;
    bool saturated = false;
};

// Caller supplies fixed RX/TX storage. Recovery clears indices, never allocates.
// step() handles at most one cause or lifecycle operation; the adapter imposes
// the shared time budget and the four-cause limit around repeated calls.
class Foreground {
  public:
    Foreground(ForegroundIo &io, ForegroundFrame *rx, size_t rxCapacity, ForegroundFrame *tx, size_t txCapacity);
    bool begin(const ForegroundConfig &config, uint64_t nowUs);
    ACAN_FOREGROUND_RAM(driver_step) ForegroundWork step(uint64_t nowUs, bool interruptLow);
    // Pure owner-local readiness check: no SPI, allocation or clock reads.
    // Backoff and healthy idle do not need repeated no-op driver calls.
    ACAN_FOREGROUND_RAM(driver_hasWork) bool hasWork(uint64_t nowUs, bool interruptLow) const {
        if (state_ == ForegroundState::Initializing) {
            return true;
        }
        if (state_ == ForegroundState::Recovering) {
            return stage_ < 3 || nowUs >= retryAt_;
        }
        return state_ == ForegroundState::Ready &&
               (pending_ != Phase::Idle || interruptLow || txCount_ || healthPhase_ || nowUs - healthAt_ >= 10000);
    }
    ForegroundUnit lastUnit() const { return lastUnit_; }
    bool inProgress() const { return pending_ != Phase::Idle; }
    bool receive(ForegroundFrame &frame);
    bool submit(const ForegroundFrame &frame, uint64_t nowUs);
    void requestRecovery(uint64_t nowUs);
    void fail(ForegroundFault fault, uint64_t nowUs);
    ForegroundState state() const {
        return state_;
    }
    ForegroundFault fault() const {
        return fault_;
    }
    uint32_t generation() const {
        return generation_;
    }
    uint32_t rxCount() const {
        return rxCount_;
    }
    uint32_t txCount() const {
        return txCount_;
    }
    const ForegroundConfig &config() const {
        return config_;
    }
    const ForegroundCounters &counters() const {
        return counters_;
    }
    uint64_t lastTxWaitUs() const {
        return lastTxWaitUs_;
    }
    uint32_t txProgress() const {
        return txProgress_;
    }
    // Conservative transaction allowance. The outer deadline still measures
    // real elapsed time and reports a peripheral/framework operation overrun.
    ACAN_FOREGROUND_RAM(driver_nextStepBudgetUs) uint32_t nextStepBudgetUs() const {
        return (state_ == ForegroundState::Initializing && stage_ <= 5) ||
                       (state_ == ForegroundState::Recovering && stage_ < 3)
                   ? 130
                   : 70;
    }
    void resetHighWater() {
        counters_.rxWindowHigh = rxCount_;
        counters_.txWindowHigh = txCount_;
    }
    static uint8_t lengthFor(uint8_t dlc);
    static int dlcFor(uint8_t length);
    static uint32_t controllerBytes(uint8_t rx, uint8_t tx) {
        return 72u * (rx + tx);
    }

  private:
    enum class Phase : uint8_t {
        Idle, RxAddress, RxObject, RxAdvance, RxPublish, TxStatus,
        TxPrepare, TxObject, TxAdvance, TxComplete, FaultClear
    };
    ACAN_FOREGROUND_RAM(driver_progressTransfer) ForegroundWork progressTransfer(uint64_t nowUs);
    void scheduleClear(uint16_t address, uint8_t value, ForegroundFault fault);
    bool initializeStep(uint64_t nowUs);
    ACAN_FOREGROUND_RAM(driver_read32) bool read32(uint16_t address, uint32_t &value);
    bool write32(uint16_t address, uint32_t value);
    ACAN_FOREGROUND_RAM(driver_write8) bool write8(uint16_t address, uint8_t value);
    bool checkedWrite(uint16_t address, uint32_t value, uint8_t width);
    void startAttempt(uint64_t nowUs);
    void clearQueues();
    ACAN_FOREGROUND_RAM(driver_add) void add(uint32_t &value, uint32_t amount = 1);
    ForegroundIo &io_;
    ForegroundFrame *rx_, *tx_;
    size_t rxCapacity_, txCapacity_;
    ForegroundConfig config_{};
    ForegroundCounters counters_{};
    ForegroundState state_ = ForegroundState::Off;
    ForegroundFault fault_ = ForegroundFault::None;
    uint32_t generation_ = 0;
    uint32_t txProgress_ = 0;
    uint16_t stage_ = 0, index_ = 0;
    uint8_t retry_ = 0, stuck_ = 0, healthPhase_ = 0;
    uint32_t rxHead_ = 0, rxCount_ = 0, txHead_ = 0, txCount_ = 0;
    uint64_t attemptAt_ = 0, modeAt_ = 0, retryAt_ = 0, healthAt_ = 0, lastTxWaitUs_ = 0;
    bool preferTx_ = false;
    Phase pending_ = Phase::Idle;
    ForegroundUnit lastUnit_ = ForegroundUnit::None;
    uint8_t pendingObject_[72]{};
    uint32_t pendingUa_ = 0;
    uint64_t pendingRxAtUs_ = 0;
    uint16_t clearAddress_ = 0;
    uint8_t clearValue_ = 0;
    ForegroundFault clearFault_ = ForegroundFault::None;
};
} // namespace acan2517fd

#include "ACAN2517FDForeground.h"
#include <limits.h>
#include <string.h>

namespace acan2517fd {
namespace {
// MCP25xxFD register map: FIFO1 RX, FIFO2 TX; TXQ and TEF disabled.
constexpr uint16_t Con = 0x000, Nbt = 0x004, Dbt = 0x008, Tdc = 0x00C;
constexpr uint16_t Vec = 0x018, Int = 0x01C, Trec = 0x034;
constexpr uint16_t RxCon = 0x05C, RxSta = 0x060, RxUa = 0x064;
constexpr uint16_t TxCon = 0x068, TxSta = 0x06C, TxUa = 0x070;
constexpr uint16_t FilterCon = 0x1D0, FilterObj = 0x1F0, FilterMask = 0x1F4;
constexpr uint16_t Osc = 0xE00, CrcFlags = 0xE0A, EccCon = 0xE0C, EccSta = 0xE10;
uint32_t le(const uint8_t *p) {
    return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
void put(uint8_t *p, uint32_t value) {
    for (unsigned i = 0; i < 4; ++i)
        p[i] = uint8_t(value >> (i * 8));
}
} // namespace

Foreground::Foreground(ForegroundIo &io, ForegroundFrame *rx, size_t rxCapacity, ForegroundFrame *tx, size_t txCapacity)
    : io_(io), rx_(rx), tx_(tx), rxCapacity_(rxCapacity), txCapacity_(txCapacity) {
}

void Foreground::add(uint32_t &value, uint32_t amount) {
    if (UINT32_MAX - value < amount) {
        value = UINT32_MAX;
        counters_.saturated = true;
    } else
        value += amount;
}
uint8_t Foreground::lengthFor(uint8_t dlc) {
    static const uint8_t lengths[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return dlc < 16 ? lengths[dlc] : 0;
}
int Foreground::dlcFor(uint8_t length) {
    for (int i = 0; i < 16; ++i)
        if (lengthFor(uint8_t(i)) == length)
            return i;
    return -1;
}
void Foreground::clearQueues() {
    rxHead_ = rxCount_ = txHead_ = txCount_ = 0;
    pending_ = Phase::Idle; // Discard any object/address on fault or generation replacement.
}
bool Foreground::read32(uint16_t address, uint32_t &value) {
    uint8_t data[4];
    if (!io_.read(address, data, sizeof(data)))
        return false;
    value = le(data);
    return true;
}
bool Foreground::write32(uint16_t address, uint32_t value) {
    uint8_t data[4];
    put(data, value);
    return io_.write(address, data, sizeof(data));
}
bool Foreground::write8(uint16_t address, uint8_t value) {
    return io_.write(address, &value, 1);
}
bool Foreground::checkedWrite(uint16_t address, uint32_t value, uint8_t width) {
    uint8_t bytes[4]{}, actual[4]{};
    put(bytes, value);
    if (!io_.write(address, bytes, width) || !io_.read(address, actual, width))
        return false;
    // OPMOD occupies the read-only upper three bits of CiCON byte 2.
    if (address == Con + 2 && width == 1)
        actual[0] &= 0x1F;
    return memcmp(bytes, actual, width) == 0;
}
bool Foreground::begin(const ForegroundConfig &config, uint64_t nowUs) {
    config_ = config;
    if (!rx_ || !tx_ || !config.softwareRx || !config.softwareTx || config.softwareRx > rxCapacity_ ||
        config.softwareTx > txCapacity_ || !config.hardwareRx || !config.hardwareTx || config.hardwareRx > 32 ||
        config.hardwareTx > 32 || controllerBytes(config.hardwareRx, config.hardwareTx) > 2048 ||
        config.requestId > 0x7FF) {
        state_ = ForegroundState::Faulted;
        fault_ = ForegroundFault::Configuration;
        return false;
    }
    retry_ = 0;
    startAttempt(nowUs);
    return true;
}
void Foreground::startAttempt(uint64_t nowUs) {
    clearQueues();
    if (++generation_ == 0)
        ++generation_;
    state_ = ForegroundState::Initializing;
    stage_ = index_ = stuck_ = 0;
    healthPhase_ = 0;
    attemptAt_ = modeAt_ = nowUs;
    io_.speed(800000);
}
void Foreground::requestRecovery(uint64_t nowUs) {
    if (fault_ == ForegroundFault::Configuration)
        return;
    retry_ = 0;
    startAttempt(nowUs);
}
void Foreground::fail(ForegroundFault reason, uint64_t nowUs) {
    // Fault transitions are idempotent until the next attempt. Purge software
    // immediately; abort/mask the controller as the first bounded recovery step.
    if (state_ == ForegroundState::Recovering || state_ == ForegroundState::Faulted)
        return;
    fault_ = reason;
    add(counters_.faults);
    if (reason == ForegroundFault::BusOff)
        add(counters_.busOff);
    if (reason == ForegroundFault::AttemptsExhausted)
        add(counters_.attemptsExhausted);
    clearQueues();
    if (++generation_ == 0)
        ++generation_;
    state_ = ForegroundState::Recovering;
    stage_ = 0;
    static const uint32_t backoff[3] = {100000, 500000, 2000000};
    retryAt_ = nowUs + (retry_ < 3 ? backoff[retry_] : 0);
}

bool Foreground::initializeStep(uint64_t nowUs) {
    if (nowUs - attemptAt_ >= 500000) {
        fail(ForegroundFault::InitializationTimeout, nowUs);
        return true;
    }
    bool ok = true;
    uint32_t value = 0;
    switch (stage_) {
    case 0:
        ok = write8(Con + 3, 0x0C); // Configuration + abort all transmissions
        modeAt_ = nowUs;
        ++stage_;
        break;
    case 1:
        ok = read32(Con, value);
        if (ok && ((value >> 21) & 7) == 4)
            ++stage_;
        else if (nowUs - modeAt_ >= 10000) {
            fail(ForegroundFault::ModeTimeout, nowUs);
            return true;
        }
        break;
    case 2:
        ok = io_.reset();
        ++stage_;
        break;
    case 3:
        ok = write8(Osc, 0x60);
        modeAt_ = nowUs;
        ++stage_;
        break;
    case 4:
        ok = read32(Osc, value);
        if (ok && (value & (1u << 10))) {
            ++stage_;
            index_ = 0;
        } else if (nowUs - modeAt_ >= 10000) {
            fail(ForegroundFault::ModeTimeout, nowUs);
            return true;
        }
        break;
    case 5: // Walking bit readback at bootstrap SPI clock; no heap or waits.
    case 6:
        ok = checkedWrite(0x400, uint32_t(1) << index_, 4);
        if (++index_ == 32) {
            index_ = 0;
            if (stage_ == 5)
                io_.speed(16000000);
            ++stage_;
        }
        break;
    case 7:
        ok = write8(EccCon, 7);
        ++stage_;
        break;
    case 8: {
        uint8_t zero[16]{};
        ok = io_.write(uint16_t(0x400 + index_), zero, sizeof(zero));
        index_ += 16;
        if (index_ == 2048) {
            index_ = 0;
            ++stage_;
        }
        break;
    }
    case 9: // Disable all filters before modifying filter objects.
        ok = checkedWrite(uint16_t(FilterCon + index_), 0, 1);
        if (++index_ == 32) {
            index_ = 0;
            ++stage_;
        }
        break;
    case 10: {
        struct Setting {
            uint16_t address;
            uint8_t width;
            uint32_t value;
        };
        // Fixed 40 MHz / 500 kbit/s / 2 Mbit/s, 80% sample points. IOCON
        // writes are single-byte. Preserve reset GPIO input/standby wiring.
        const Setting settings[] = {{0xE07, 1, 3},
                                    {Con, 1, 0x60},
                                    {Con + 2, 1, 1},
                                    {Nbt, 4, 0x003E0F0F},
                                    {Dbt, 4, 0x000E0303},
                                    {Tdc, 4, 0x02020F00},
                                    {RxCon + 3, 1, uint32_t(0xE0 | (config_.hardwareRx - 1))},
                                    {RxCon, 1, 9},
                                    {TxCon + 3, 1, uint32_t(0xE0 | (config_.hardwareTx - 1))},
                                    {TxCon + 2, 1, 0x20},
                                    {TxCon, 1, 0x90},
                                    {FilterObj, 4, config_.requestId},
                                    {FilterMask, 4, 0x400007FF},
                                    {FilterCon, 1, 0x81},
                                    {CrcFlags + 1, 1, 3},
                                    {Int + 2, 1, 0x0A},
                                    {Int + 3, 1, 0x3F}};
        const auto &entry = settings[index_];
        ok = checkedWrite(entry.address, entry.value, entry.width);
        if (++index_ == sizeof(settings) / sizeof(settings[0])) {
            index_ = 0;
            ++stage_;
        }
        break;
    }
    case 11:
        ok = write8(Con + 3, config_.loopback ? 2 : 0);
        modeAt_ = nowUs;
        ++stage_;
        break;
    case 12:
        ok = read32(Con, value);
        if (ok && ((value >> 21) & 7) == unsigned(config_.loopback ? 2 : 0)) {
            state_ = ForegroundState::Ready;
            healthAt_ = nowUs;
            if (retry_)
                add(counters_.recoveries);
            retry_ = 0;
        } else if (nowUs - modeAt_ >= 10000) {
            fail(ForegroundFault::ModeTimeout, nowUs);
            return true;
        }
        break;
    default:
        ok = false;
        break;
    }
    if (!ok)
        fail(ForegroundFault::Spi, nowUs);
    return true;
}

void Foreground::scheduleClear(uint16_t address, uint8_t value, ForegroundFault fault) {
    clearAddress_ = address;
    clearValue_ = value;
    clearFault_ = fault;
    pending_ = Phase::FaultClear;
}

ForegroundWork Foreground::progressTransfer(uint64_t nowUs) {
    switch (pending_) {
    case Phase::RxAddress:
        lastUnit_ = ForegroundUnit::RxAddress;
        if (!read32(RxUa, pendingUa_) || pendingUa_ > 1976) {
            fail(ForegroundFault::Spi, nowUs);
        } else {
            pending_ = Phase::RxObject;
        }
        return ForegroundWork::Receive;
    case Phase::RxObject:
        lastUnit_ = ForegroundUnit::RxObject;
        if (!io_.read(uint16_t(0x400 + pendingUa_), pendingObject_, sizeof(pendingObject_))) {
            fail(ForegroundFault::Spi, nowUs);
        } else {
            pending_ = Phase::RxAdvance;
        }
        return ForegroundWork::Receive;
    case Phase::RxAdvance:
        lastUnit_ = ForegroundUnit::RxAdvance;
        if (!write8(RxCon + 1, 1)) {
            fail(ForegroundFault::Spi, nowUs);
        } else {
            pendingRxAtUs_ = io_.timestampUs();
            pending_ = Phase::RxPublish;
        }
        return ForegroundWork::Receive;
    case Phase::RxPublish: {
        lastUnit_ = ForegroundUnit::RxPublish;
        ForegroundFrame frame{};
        const uint32_t header = le(pendingObject_ + 4);
        frame.id = le(pendingObject_) & 0x7FF;
        frame.length = lengthFor(uint8_t(header & 15));
        frame.flags = uint8_t(((header >> 7) & 1) | (((header >> 6) & 1) << 1) |
                              (((header >> 4) & 1) << 2) | (((header >> 5) & 1) << 3));
        memcpy(frame.data, pendingObject_ + 8, frame.length);
        frame.queuedUs = pendingRxAtUs_;
        add(counters_.rxFrames);
        add(counters_.rxBytes, frame.length);
        if (rxCount_ == config_.softwareRx) {
            add(counters_.softwareOverflow);
        } else {
            rx_[(rxHead_ + rxCount_) % config_.softwareRx] = frame;
            ++rxCount_;
            if (counters_.rxHigh < rxCount_) counters_.rxHigh = rxCount_;
            if (counters_.rxWindowHigh < rxCount_) counters_.rxWindowHigh = rxCount_;
        }
        pending_ = Phase::Idle;
        preferTx_ = true;
        stuck_ = 0;
        return ForegroundWork::Receive;
    }
    case Phase::TxStatus: {
        lastUnit_ = ForegroundUnit::TxStatusAddress;
        preferTx_ = false; // Full TX must leave the next RX eligible.
        uint8_t registers[8]{};
        if (!io_.read(TxSta, registers, sizeof(registers))) {
            fail(ForegroundFault::Spi, nowUs);
        } else {
            const uint32_t status = le(registers);
            pending_ = Phase::Idle;
            if (status & 16) {
                scheduleClear(TxSta, 0xEF, ForegroundFault::AttemptsExhausted);
            } else if (status & 1) {
                pendingUa_ = le(registers + 4);
                if (pendingUa_ > 1976) fail(ForegroundFault::Spi, nowUs);
                else pending_ = Phase::TxPrepare;
            }
        }
        return ForegroundWork::Transmit;
    }
    case Phase::TxPrepare: {
        lastUnit_ = ForegroundUnit::TxPrepare;
        const auto &frame = tx_[txHead_];
        memset(pendingObject_, 0, sizeof(pendingObject_));
        put(pendingObject_, frame.id);
        put(pendingObject_ + 4, uint32_t(dlcFor(frame.length)) | 0xC0);
        memcpy(pendingObject_ + 8, frame.data, frame.length);
        pending_ = Phase::TxObject;
        return ForegroundWork::Transmit;
    }
    case Phase::TxObject:
        lastUnit_ = ForegroundUnit::TxObject;
        if (!io_.write(uint16_t(0x400 + pendingUa_), pendingObject_,
                       (8 + tx_[txHead_].length + 3) & ~size_t(3))) {
            fail(ForegroundFault::Spi, nowUs);
        } else {
            pending_ = Phase::TxAdvance;
        }
        return ForegroundWork::Transmit;
    case Phase::TxAdvance:
        lastUnit_ = ForegroundUnit::TxAdvance;
        if (!write8(TxCon + 1, 3)) {
            // Ambiguous transfer completion is never retried in this generation.
            fail(ForegroundFault::Spi, nowUs);
        } else {
            lastTxWaitUs_ = io_.timestampUs() - tx_[txHead_].queuedUs;
            pending_ = Phase::TxComplete;
        }
        return ForegroundWork::Transmit;
    case Phase::TxComplete:
        lastUnit_ = ForegroundUnit::TxComplete;
        ++txProgress_;
        txHead_ = (txHead_ + 1) % config_.softwareTx;
        --txCount_;
        pending_ = Phase::Idle;
        return ForegroundWork::Transmit;
    case Phase::FaultClear: {
        lastUnit_ = ForegroundUnit::FaultClear;
        const auto fault = clearFault_;
        pending_ = Phase::Idle;
        if (!write8(clearAddress_, clearValue_)) fail(ForegroundFault::Spi, nowUs);
        else if (fault != ForegroundFault::None) fail(fault, nowUs);
        return ForegroundWork::Maintenance;
    }
    default:
        return ForegroundWork::None;
    }
}

ForegroundWork Foreground::step(uint64_t nowUs, bool interruptLow) {
    lastUnit_ = ForegroundUnit::None;
    if (state_ == ForegroundState::Off || state_ == ForegroundState::Faulted)
        return ForegroundWork::None;
    if (state_ == ForegroundState::Recovering) {
        lastUnit_ = ForegroundUnit::Recovery;
        if (stage_ < 3) {
            // Mask/abort uses one SPI transaction per call, not a three-write burst.
            if (stage_ == 0) write8(Int + 2, 0);
            else if (stage_ == 1) write8(Int + 3, 0);
            else write8(Con + 3, 0x0C);
            ++stage_;
        } else if (nowUs >= retryAt_) {
            if (retry_ >= 3) state_ = ForegroundState::Faulted;
            else {
                ++retry_;
                add(counters_.recoveryAttempts);
                startAttempt(nowUs);
            }
        }
        return ForegroundWork::Recovery;
    }
    if (state_ == ForegroundState::Initializing) {
        lastUnit_ = ForegroundUnit::Initialization;
        initializeStep(nowUs);
        return ForegroundWork::Initialization;
    }
    // UA is retained only for this single in-flight object. This owner is the
    // sole consumer/producer; its UA cannot advance until our UINC. Every
    // recovery/begin/fault clears pending_, so it never survives a generation.
    if (pending_ != Phase::Idle) return progressTransfer(nowUs);
    uint32_t value = 0;
    if (!interruptLow) stuck_ = 0;
    if (healthPhase_ || nowUs - healthAt_ >= 10000) {
        lastUnit_ = ForegroundUnit::Health;
        const uint16_t address = healthPhase_ == 0 ? Trec : healthPhase_ == 1 ? Con : Osc;
        if (!healthPhase_) healthAt_ = nowUs;
        if (!read32(address, value)) fail(ForegroundFault::Spi, nowUs);
        else if (!healthPhase_) {
            counters_.trec = value;
            if (value & (1u << 21)) fail(ForegroundFault::BusOff, nowUs);
        } else if (healthPhase_ == 1) {
            if (value == UINT32_MAX) fail(ForegroundFault::Spi, nowUs);
            else if (((value >> 21) & 7) != unsigned(config_.loopback ? 2 : 0))
                fail(ForegroundFault::System, nowUs);
        } else if ((value & 0x47F) != 0x460) fail(ForegroundFault::Spi, nowUs);
        healthPhase_ = uint8_t((healthPhase_ + 1) % 3);
        return ForegroundWork::Maintenance;
    }
    if (interruptLow) {
        lastUnit_ = ForegroundUnit::VectorStatus;
        uint8_t block[8]{};
        if (!io_.read(Vec, block, sizeof(block))) {
            fail(ForegroundFault::Spi, nowUs);
            return ForegroundWork::Maintenance;
        }
        counters_.vector = le(block);
        const uint16_t flags = uint16_t(le(block + 4));
        if (flags == 0xFFFF) fail(ForegroundFault::Spi, nowUs);
        else if (flags & (1u << 8)) scheduleClear(EccSta, 0xF9, ForegroundFault::Ecc);
        else if (flags & (1u << 9)) scheduleClear(CrcFlags, 0xFC, ForegroundFault::SpiCrc);
        else if (flags & (1u << 10)) scheduleClear(TxSta, 0xEF, ForegroundFault::AttemptsExhausted);
        else if (flags & (1u << 11)) {
            add(counters_.hardwareOverflow);
            scheduleClear(RxSta, 0xF7, ForegroundFault::None);
        } else if (flags & ((1u << 12) | (1u << 13) | (1u << 15))) {
            scheduleClear(Int + 1, uint8_t(~((flags >> 8) & 0xB0)), ForegroundFault::System);
        } else if (flags & (1u << 3)) scheduleClear(Int, 0xF7, ForegroundFault::None);
        else if ((flags & 2) && (!preferTx_ || !txCount_)) {
            pending_ = Phase::RxAddress;
            return ForegroundWork::Receive;
        } else {
            if (!(flags & 2) && ++stuck_ >= 8) fail(ForegroundFault::PersistentInterrupt, nowUs);
            if (state_ == ForegroundState::Ready && txCount_) pending_ = Phase::TxStatus;
        }
        return ForegroundWork::Maintenance;
    }
    if (txCount_) {
        pending_ = Phase::TxStatus;
        return progressTransfer(nowUs);
    }
    preferTx_ = false;
    return ForegroundWork::None;
}

bool Foreground::receive(ForegroundFrame &frame) {
    if (!rxCount_)
        return false;
    frame = rx_[rxHead_];
    rxHead_ = (rxHead_ + 1) % config_.softwareRx;
    --rxCount_;
    return true;
}
bool Foreground::submit(const ForegroundFrame &frame, uint64_t nowUs) {
    if (state_ != ForegroundState::Ready || txCount_ == config_.softwareTx || frame.id > 0x7FF || frame.flags != 3 ||
        dlcFor(frame.length) < 0) {
        add(counters_.txRejected);
        return false;
    }
    tx_[(txHead_ + txCount_) % config_.softwareTx] = frame;
    tx_[(txHead_ + txCount_) % config_.softwareTx].queuedUs = nowUs;
    ++txCount_;
    if (counters_.txHigh < txCount_)
        counters_.txHigh = txCount_;
    if (counters_.txWindowHigh < txCount_)
        counters_.txWindowHigh = txCount_;
    add(counters_.txFrames);
    add(counters_.txBytes, frame.length);
    return true;
}
} // namespace acan2517fd

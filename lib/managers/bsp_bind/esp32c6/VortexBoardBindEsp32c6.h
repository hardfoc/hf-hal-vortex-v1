/**
 * @file VortexBoardBindEsp32c6.h
 * @brief ESP32-C6-only accessors of the Vortex board binding, for platform code above the HAL.
 *
 * @details The managers are MCU-free and hand out only `Base*` objects. Some platform code needs the
 *          concrete ESP32 object behind one of them: the CANopen device builds hf-core's
 *          `hf::can::EspTwaiPort` on the TWAI controller, and that port takes an `EspCan&`. This header is
 *          where such code gets it. Include it only from ESP32-C6 platform code (the product's CANopen
 *          bring-up), never from a manager or from MCU-neutral application logic.
 *
 *          @code
 *          #include "bsp_bind/esp32c6/VortexBoardBindEsp32c6.h"
 *          #include "managers/CommChannelsManager.h"
 *
 *          auto& comms = Vortex::GetInstance().comms;          // after EnsureInitialized()
 *          EspCan* twai = vortex::bsp::esp32c6::CanController();
 *          if (twai != nullptr) {
 *              hf::can::EspTwaiPort port(*twai, comms.GetCanBitRate());
 *          }
 *          @endcode
 *
 *          Ownership: the binding owns the object (static storage); it stays valid until
 *          CommChannelsManager is de-initialised (vortex::bsp::CanClose()).
 *          Thread safety: the call itself reads one pointer; the EspCan has its own locking.
 */

#ifndef VORTEX_BOARD_BIND_ESP32C6_H_
#define VORTEX_BOARD_BIND_ESP32C6_H_

class EspCan;

namespace vortex::bsp::esp32c6 {

/**
 * @brief The TWAI controller CommChannelsManager opened, as its concrete EspCan.
 * @return The same object as `CommChannelsManager::GetCanBus()`, or nullptr if CAN is not open.
 */
EspCan* CanController() noexcept;

}  // namespace vortex::bsp::esp32c6

#endif  // VORTEX_BOARD_BIND_ESP32C6_H_

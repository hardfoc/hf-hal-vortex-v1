#include "CommChannelsManager.h"

// ESP32 comm interface includes
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspSpi.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspI2c.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspUart.h"
#include "core/hf-core-drivers/internal/hf-internal-interface-wrap/inc/mcu/esp32/EspCan.h"

// Logger for unified logging
#include "handlers/Logger.h"

// Board mapping includes (already included in header)

CommChannelsManager& CommChannelsManager::GetInstance() noexcept {
    static CommChannelsManager instance;
    return instance;
}

CommChannelsManager::CommChannelsManager() {
    // Initialize vectors for managing multiple buses
    spi_device_indices_.reserve(4); // Reserve space for 4 SPI devices
    i2c_device_indices_.reserve(4); // Reserve space for 4 I2C devices
    i2c_device_addresses_.reserve(4); // Reserve space for 4 I2C device addresses
    uart_buses_.reserve(1);  // Reserve space for 1 UART bus
    can_buses_.reserve(1);   // Reserve space for 1 CAN bus
    
    // Custom device maps are automatically initialized as empty
}

bool CommChannelsManager::Initialize() noexcept {

    //================ UART FOR TMC9660 TMCL COMMUNICATION =================//
    /*
    * # UART Configuration for TMC9660
    * - UART0 configured for TMC9660 TMCL protocol communication
    * - Baud rate: 115200, Format: 8N1 (8 data bits, no parity, 1 stop bit)
    * - Buffers: TX=256 bytes, RX=512 bytes (optimized for 9-byte TMCL frames)
    */
    {
        Logger::GetInstance().Info("CommChannelsManager", "Configuring UART for TMC9660 TMCL communication");
        
        hf_uart_config_t uart_config = {};
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::UART_TXD);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::UART_RXD);
        
        if (tx_map && rx_map) {
            // ESP-IDF v5.5 compliant UART configuration for TMC9660 TMCL protocol
            uart_config.port_number = 0;                                           // UART0 port
            uart_config.baud_rate = 115200;                                        // TMC9660 standard baud rate
            uart_config.data_bits = hf_uart_data_bits_t::HF_UART_DATA_8_BITS;      // 8 data bits per TMCL spec
            uart_config.parity = hf_uart_parity_t::HF_UART_PARITY_DISABLE;         // No parity for TMCL protocol
            uart_config.stop_bits = hf_uart_stop_bits_t::HF_UART_STOP_BITS_1;      // 1 stop bit
            uart_config.flow_control = hf_uart_flow_ctrl_t::HF_UART_HW_FLOWCTRL_DISABLE; // No flow control
            uart_config.tx_pin = tx_map->physical_pin;                             // TX: GPIO5 (UART_TXD)
            uart_config.rx_pin = rx_map->physical_pin;                             // RX: GPIO4 (UART_RXD)
            uart_config.rts_pin = HF_UART_IO_UNUSED;                               // RTS not used
            uart_config.cts_pin = HF_UART_IO_UNUSED;                               // CTS not used
            
            // Buffer configuration optimized for TMC9660 9-byte TMCL datagrams
            uart_config.tx_buffer_size = 256;                                      // TX buffer (~28 TMCL frames)
            uart_config.rx_buffer_size = 512;                                      // RX buffer (~56 TMCL frames)
            uart_config.event_queue_size = 20;                                     // Event queue for interrupts
            uart_config.operating_mode = hf_uart_operating_mode_t::HF_UART_MODE_INTERRUPT; // Interrupt-driven
            uart_config.timeout_ms = 100;                                          // Timeout for operations
            uart_config.enable_pattern_detection = false;                          // Disable for now
            uart_config.enable_wakeup = false;                                     // No wakeup needed
            uart_config.enable_loopback = false;                                   // No loopback testing
            
            // Create UART bus
            uart_buses_.emplace_back(std::make_unique<EspUart>(uart_config));
            
            Logger::GetInstance().Info("CommChannelsManager", "UART configured for TMC9660: 115200 8N1, TX=GPIO%d, RX=GPIO%d", 
                     tx_map->physical_pin, rx_map->physical_pin);
            Logger::GetInstance().Info("CommChannelsManager", "UART buffers: TX=%d bytes, RX=%d bytes (optimized for 9-byte TMCL frames)",
                     uart_config.tx_buffer_size, uart_config.rx_buffer_size);
        } else {
            Logger::GetInstance().Error("CommChannelsManager", "UART GPIO mapping missing - TMC9660 communication will not be available");
            Logger::GetInstance().Error("CommChannelsManager", "Required pins: UART_TXD=%s, UART_RXD=%s", 
                     tx_map ? "OK" : "MISSING", rx_map ? "OK" : "MISSING");
        }
    }

    //================ SPI2_HOST =================//
    {
        hf_spi_bus_config_t spi_config = {};

        // Get pin mappings for SPI
        // Note: This assumes SPI2_MISO, SPI2_MOSI, SPI2_SCK are defined in HfFunctionalGpioPin
        auto* miso_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_MISO);
        auto* mosi_map = GetGpioMapping(HfFunctionalGpioPin::SPI2_MOSI);
        auto* sck_map  = GetGpioMapping(HfFunctionalGpioPin::SPI2_SCK);

        // If all required pins are mapped to a proper structure, create the SPI bus
        if (miso_map && mosi_map && sck_map) {
            spi_config.host = static_cast<hf_spi_host_device_t>(1); // SPI2_HOST
            spi_config.miso_pin = miso_map->physical_pin;
            spi_config.mosi_pin = mosi_map->physical_pin;
            spi_config.sclk_pin = sck_map->physical_pin;
            spi_config.dma_channel = 0; // Auto-select DMA channel
            spi_config.timeout_ms = 1000;
            spi_config.use_iomux = true; // Use IOMUX for better performance

            // Create the bus
            spi_bus_ = std::make_unique<EspSpiBus>(spi_config);
            spi_bus_->Initialize();

            // Now create devices for all configured CS pins
            for (auto cs_pin_enum : kSpiCsPins) {
                auto* cs_map = GetGpioMapping(cs_pin_enum);
                if (cs_map) {
                    hf_spi_device_config_t dev_cfg = {};
                    dev_cfg.cs_pin = cs_map->physical_pin;
                    dev_cfg.clock_speed_hz = 10 * 1000000; // 10 MHz max for both devices
                    
                    // Configure SPI mode based on device type
                    if (cs_pin_enum == HfFunctionalGpioPin::SPI2_CS_AS5047) {
                        // AS5047U: SPI Mode 1 (CPOL=0, CPHA=1) - samples on falling CLK edge
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_1;
                    } else if (cs_pin_enum == HfFunctionalGpioPin::SPI2_CS_TMC9660) {
                        // TMC9660: SPI Mode 3 (CPOL=1, CPHA=1) - idle high, sample on falling edge
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_3;
                    } else {
                        // Default mode for external GPIO devices
                        dev_cfg.mode = hf_spi_mode_t::HF_SPI_MODE_0;
                    }
                    
                    dev_cfg.queue_size = 7;
                    dev_cfg.flags = 0;
                    dev_cfg.cs_ena_pretrans = 0;
                    dev_cfg.cs_ena_posttrans = 0;
                    dev_cfg.duty_cycle_pos = 128;
                    dev_cfg.input_delay_ns = 0;
                    dev_cfg.command_bits = 0;
                    dev_cfg.address_bits = 0;
                    dev_cfg.dummy_bits = 0;
                    dev_cfg.pre_cb = nullptr;
                    dev_cfg.post_cb = nullptr;
                    
                    // Create device and store its index
                    int device_index = spi_bus_->CreateDevice(dev_cfg);
                    if (device_index >= 0) {
                        spi_device_indices_.push_back(device_index);
                    }
                }
            }
        }
    }

    //================ I2C FOR BNO08X IMU AND PCAL95555 GPIO EXPANDER =================//
    {
        Logger::GetInstance().Info("CommChannelsManager", "Configuring I2C master bus for BNO08x IMU and PCAL95555 GPIO expander");
        
        hf_i2c_master_bus_config_t i2c_config = {};
        auto* sda_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SDA);
        auto* scl_map = GetGpioMapping(HfFunctionalGpioPin::I2C_SCL);
        
        if (sda_map && scl_map) {
            // ESP-IDF v5.5+ I2C master bus configuration for BNO08x and PCAL95555
            i2c_config.i2c_port = I2C_NUM_0;                                               // I2C port 0 (ESP32C6 has 1 I2C port)
            i2c_config.sda_io_num = sda_map->physical_pin;                                 // SDA GPIO pin from board mapping
            i2c_config.scl_io_num = scl_map->physical_pin;                                 // SCL GPIO pin from board mapping
            i2c_config.enable_internal_pullup = sda_map->has_pullup && scl_map->has_pullup; // Use internal pullups if available
            i2c_config.clk_source = hf_i2c_clock_source_t::HF_I2C_CLK_SRC_DEFAULT;        // Default clock source (APB)
            i2c_config.clk_flags = 0;                                                      // No additional clock flags
            i2c_config.glitch_ignore_cnt = hf_i2c_glitch_filter_t::HF_I2C_GLITCH_FILTER_7_CYCLES; // 7-cycle glitch filter for noise immunity
            i2c_config.trans_queue_depth = 8;                                              // Transaction queue depth for async operations
            i2c_config.intr_priority = 5;                                                  // Interrupt priority (0-7, 5=medium)
            i2c_config.flags = 0;                                                          // No additional configuration flags
            i2c_config.allow_pd = false;                                                   // Disable power down in sleep modes
            
            // Create I2C master bus instance
            i2c_bus_ = std::make_unique<EspI2cBus>(i2c_config);
            
            // Initialize the I2C bus
            if (i2c_bus_->Initialize()) {
                Logger::GetInstance().Info("CommChannelsManager", "I2C master bus initialized: SDA=GPIO%d, SCL=GPIO%d", 
                         sda_map->physical_pin, scl_map->physical_pin);
                Logger::GetInstance().Info("CommChannelsManager", "I2C configuration: 400kHz, 7-cycle glitch filter, queue depth=8");
                
                // Add BNO08x IMU device (standard I2C address 0x4A)
                {
                    hf_i2c_device_config_t bno08x_device = {};
                    bno08x_device.device_address = 0x4A;                                       // BNO08x standard I2C address
                    bno08x_device.dev_addr_length = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT;  // 7-bit addressing
                    bno08x_device.scl_speed_hz = 400000;                                       // 400kHz for fast I2C operation
                    bno08x_device.scl_wait_us = 0;                                             // No additional SCL wait time
                    bno08x_device.disable_ack_check = false;                                   // Enable ACK check
                    bno08x_device.flags = 0;                                                   // No device-specific flags
                    
                    int device_index = i2c_bus_->CreateDevice(bno08x_device);
                    if (device_index >= 0) {
                        i2c_device_indices_.push_back(device_index);
                        i2c_device_addresses_.push_back(bno08x_device.device_address);
                        i2c_device_external_.push_back(false); // Built-in device (cannot be removed)
                        Logger::GetInstance().Info("CommChannelsManager", "BNO08x IMU device added to I2C bus: address=0x%02X, speed=400kHz", 
                                 bno08x_device.device_address);
                    } else {
                                                 Logger::GetInstance().Warn("CommChannelsManager", "Failed to add BNO08x IMU device to I2C bus");
                    }
                }
                
                // Add PCAL95555 GPIO expander device (standard I2C address range 0x20-0x27)
                {
                    hf_i2c_device_config_t pcal95555_device = {};
                    pcal95555_device.device_address = 0x20;                                    // PCAL95555 base address (A0=A1=A2=0)
                    pcal95555_device.dev_addr_length = hf_i2c_address_bits_t::HF_I2C_ADDR_7_BIT; // 7-bit addressing
                    pcal95555_device.scl_speed_hz = 400000;                                    // 400kHz for fast I2C operation
                    pcal95555_device.scl_wait_us = 0;                                          // No additional SCL wait time
                    pcal95555_device.disable_ack_check = false;                                // Enable ACK check
                    pcal95555_device.flags = 0;                                                // No device-specific flags
                    
                    int device_index = i2c_bus_->CreateDevice(pcal95555_device);
                    if (device_index >= 0) {
                        i2c_device_indices_.push_back(device_index);
                        i2c_device_addresses_.push_back(pcal95555_device.device_address);
                        i2c_device_external_.push_back(false); // Built-in device (cannot be removed)
                        Logger::GetInstance().Info("CommChannelsManager", "PCAL95555 GPIO expander device added to I2C bus: address=0x%02X, speed=400kHz", 
                                 pcal95555_device.device_address);
                    } else {
                                                 Logger::GetInstance().Warn("CommChannelsManager", "Failed to add PCAL95555 GPIO expander device to I2C bus");
                    }
                }
                
                Logger::GetInstance().Info("CommChannelsManager", "I2C bus ready for BNO08x IMU and PCAL95555 GPIO expander communication");
            } else {
                Logger::GetInstance().Error("CommChannelsManager", "Failed to initialize I2C master bus");
            }
        } else {
            Logger::GetInstance().Error("CommChannelsManager", "I2C GPIO mapping missing - I2C devices will not be available");
            Logger::GetInstance().Error("CommChannelsManager", "Required pins: I2C_SDA=%s, I2C_SCL=%s", 
                     sda_map ? "OK" : "MISSING", scl_map ? "OK" : "MISSING");
        }
    }

    //================ CAN =================//
    {
        hf_esp_can_config_t can_config = {};
        auto* tx_map = GetGpioMapping(HfFunctionalGpioPin::TWAI_TX);
        auto* rx_map = GetGpioMapping(HfFunctionalGpioPin::TWAI_RX);
        if (tx_map && rx_map) {
            can_config.controller_id = hf_can_controller_id_t::HF_CAN_CONTROLLER_0;
            can_config.tx_pin = tx_map->physical_pin;
            can_config.rx_pin = rx_map->physical_pin;
            can_config.baud_rate = 500000;
            // ... set other config fields as needed
            can_buses_.emplace_back(std::make_unique<EspCan>(can_config));
        } else {
            // TODO: Log warning: CAN mapping missing, not instantiating CAN bus
        }
    }

    //================ Initialization =================//

    // If no buses were created, return false
    if (!spi_bus_ && !i2c_bus_ && uart_buses_.empty() && can_buses_.empty()) {
        // No buses initialized, return false
        return false;
    }

    // Track if all buses were successfully initialized
    bool all_initialized = true;

    // Initialize SPI bus
    if (spi_bus_ && !spi_bus_->Initialize()) {
        all_initialized = false;
    }

    // Initialize I2C bus (already initialized in the configuration block above)
    // The I2C bus is initialized when created and devices are added
    for (auto& uart : uart_buses_) {
        if (!uart->EnsureInitialized()) {
            all_initialized = false;
        }
    }
    for (auto& can : can_buses_) {
        if (!can->EnsureInitialized()) {
            all_initialized = false;
        }
    }

    // Register built-in devices with proper bus associations
    if (all_initialized) {
        RegisterBuiltinDevices();
    }

    return all_initialized;
}

bool CommChannelsManager::Deinitialize() noexcept {
    if (!initialized_) {
        return true; // Already deinitialized
    }
    
    // If no buses were created, return false
    if (!spi_bus_ && !i2c_bus_ && uart_buses_.empty() && can_buses_.empty()) {
        // No buses initialized, return false
        return false;
    }

    // Track if all buses were successfully deinitialized
    bool all_deinitialized = true;

    // Deinitialize SPI bus
    if (spi_bus_ && !spi_bus_->Deinitialize()) {
        all_deinitialized = false;
    }

    // Deinitialize I2C bus
    if (i2c_bus_ && !i2c_bus_->Deinitialize()) {
        all_deinitialized = false;
    }
    for (auto& uart : uart_buses_) {
        if (!uart->EnsureDeinitialized()) {
            all_deinitialized = false;
        }
    }
    for (auto& can : can_buses_) {
        if (!can->EnsureDeinitialized()) {
            all_deinitialized = false;
        }
    }

    return all_deinitialized;
}

CommChannelsManager::~CommChannelsManager() = default;

//==================== Accessors ====================//

BaseSpi* CommChannelsManager::GetSpiDevice(uint8_t bus_index, int device_index) noexcept {
    if (device_index < 0) {
        return nullptr;
    }
    
    // Handle built-in ESP32 bus (Bus 0)
    if (bus_index == 0) {
        if (!spi_bus_) {
            return nullptr;
        }
        
        // Check if this is a built-in device
        auto index_iter = std::find(spi_device_indices_.begin(), spi_device_indices_.end(), device_index);
        if (index_iter != spi_device_indices_.end()) {
            return spi_bus_->GetDevice(device_index);
        }
        
        return nullptr;
    }
    
    // Handle external buses (Bus 10+)
    if (bus_index >= 10) {
        // Check if this device belongs to the specified external bus
        auto mapping_iter = spi_device_to_bus_mapping_.find(device_index);
        if (mapping_iter != spi_device_to_bus_mapping_.end() && 
            mapping_iter->second == bus_index) {
            
            // Return the custom device
            auto custom_iter = custom_spi_devices_.find(device_index);
            if (custom_iter != custom_spi_devices_.end()) {
                return custom_iter->second.get();
            }
        }
    }
    
    return nullptr;
}

BaseSpi* CommChannelsManager::GetSpiDevice(SpiDeviceId device_id) noexcept {
    // Map device ID to known bus/device location established during initialization
    switch (device_id) {
        case SpiDeviceId::TMC9660_MOTOR_CONTROLLER:
            return GetSpiDevice(0, static_cast<int>(SpiDeviceId::TMC9660_MOTOR_CONTROLLER));
        case SpiDeviceId::AS5047U_POSITION_ENCODER:
            return GetSpiDevice(0, static_cast<int>(SpiDeviceId::AS5047U_POSITION_ENCODER));
        case SpiDeviceId::EXTERNAL_DEVICE_1:
            return GetSpiDevice(0, static_cast<int>(SpiDeviceId::EXTERNAL_DEVICE_1));
        case SpiDeviceId::EXTERNAL_DEVICE_2:
            return GetSpiDevice(0, static_cast<int>(SpiDeviceId::EXTERNAL_DEVICE_2));
        default:
            return nullptr;
    }
}

//==================== I2C Accessors ====================//

BaseI2c* CommChannelsManager::GetI2cDevice(uint8_t bus_index, int device_index) noexcept {
    if (device_index < 0) {
        return nullptr;
    }
    
    // Handle built-in ESP32 bus (Bus 0)
    if (bus_index == 0) {
        if (!i2c_bus_) {
            return nullptr;
        }
        
        // Check if this is a built-in device
        auto index_iter = std::find(i2c_device_indices_.begin(), i2c_device_indices_.end(), device_index);
        if (index_iter != i2c_device_indices_.end()) {
            return i2c_bus_->GetDevice(device_index);
        }
        
        return nullptr;
    }
    
    // Handle external buses (Bus 10+)
    if (bus_index >= 10) {
        // Check if this device belongs to the specified external bus
        auto mapping_iter = i2c_device_to_bus_mapping_.find(device_index);
        if (mapping_iter != i2c_device_to_bus_mapping_.end() && 
            mapping_iter->second == bus_index) {
            
            // Return the custom device
            auto custom_iter = custom_i2c_devices_.find(device_index);
            if (custom_iter != custom_i2c_devices_.end()) {
                return custom_iter->second.get();
            }
        }
    }
    
    return nullptr;
}

BaseI2c* CommChannelsManager::GetI2cDevice(I2cDeviceId device_id) noexcept {
    // Map device ID to known bus/device location established during initialization
    switch (device_id) {
        case I2cDeviceId::BNO08X_IMU:
            return GetI2cDevice(0, static_cast<int>(I2cDeviceId::BNO08X_IMU));
        case I2cDeviceId::PCAL9555_GPIO_EXPANDER:
            return GetI2cDevice(0, static_cast<int>(I2cDeviceId::PCAL9555_GPIO_EXPANDER));
        default:
            return nullptr;
    }
}

//==================== RUNTIME I2C DEVICE MANAGEMENT ====================//

int CommChannelsManager::CreateI2cDevice(uint8_t device_address, uint32_t speed_hz) noexcept {
    // Create device on built-in bus (Bus 0)
    return CreateI2cDevice(0, device_address, speed_hz);
}

int CommChannelsManager::CreateI2cDevice(uint8_t bus_index, uint8_t device_address, uint32_t speed_hz) noexcept {
    if (!i2c_bus_ || bus_index != 0) {
        Logger::GetInstance().Error("CommChannelsManager", "Cannot create I2C device: bus not available or invalid bus index");
        return -1;
    }
    
    // Check if device already exists at this address on the specified bus
    if (HasI2cDeviceAtAddress(bus_index, device_address)) {
                 Logger::GetInstance().Warn("CommChannelsManager", "I2C device already exists at address 0x%02X on bus %d", device_address, bus_index);
        return GetI2cDeviceIndexByAddress(device_address);
    }
    
    // Get next available device index
    int device_index = GetNextAvailableDeviceIndex(bus_index);
    if (device_index == -1) {
        Logger::GetInstance().Error("CommChannelsManager", "No available I2C device slots");
        return -1;
    }
    
    // Create device configuration
    hf_i2c_device_config_t device_config = {
        .device_address = device_address,
        .speed_hz = speed_hz,
        .sda_pin = kI2cSdaPin,
        .scl_pin = kI2cSclPin,
        .timeout_ms = 1000
    };
    
    // Add device to I2C bus
    if (!i2c_bus_->AddDevice(device_config)) {
        Logger::GetInstance().Error("CommChannelsManager", "Failed to add I2C device at address 0x%02X", device_address);
        return -1;
    }
    
    // Track the device
    i2c_device_indices_.push_back(device_index);
    i2c_device_addresses_.push_back(device_address);
    i2c_device_external_.push_back(true);  // Runtime-created devices are external
    i2c_device_to_bus_mapping_[device_index] = bus_index;
    
    Logger::GetInstance().Info("CommChannelsManager", "Created I2C device at address 0x%02X with index %d on bus %d", 
             device_address, device_index, bus_index);
    
    return device_index;
}

bool CommChannelsManager::RemoveI2cDevice(int device_index) noexcept {
    if (device_index < 0) {
        Logger::GetInstance().Error("CommChannelsManager", "Invalid I2C device index: %d", device_index);
        return false;
    }
    
    // Check if this is a custom I2C device (external bus)
    auto custom_iter = custom_i2c_devices_.find(device_index);
    if (custom_iter != custom_i2c_devices_.end()) {
        // Get the bus index for this device
        auto mapping_iter = i2c_device_to_bus_mapping_.find(device_index);
        if (mapping_iter != i2c_device_to_bus_mapping_.end()) {
            uint8_t bus_index = mapping_iter->second;
            
            // Remove from custom devices
            custom_i2c_devices_.erase(custom_iter);
            i2c_device_to_bus_mapping_.erase(mapping_iter);
            
            Logger::GetInstance().Info("CommChannelsManager", "Removed custom I2C device with index %d from bus %d", device_index, bus_index);
            return true;
        }
    }
    
    // Check if this is a built-in device (Bus 0)
    auto index_iter = std::find(i2c_device_indices_.begin(), i2c_device_indices_.end(), device_index);
    if (index_iter != i2c_device_indices_.end()) {
        // Check if this is an external (runtime-created) device
        size_t vector_index = std::distance(i2c_device_indices_.begin(), index_iter);
        if (vector_index < i2c_device_external_.size() && i2c_device_external_[vector_index]) {
            // Remove from I2C bus
            if (i2c_bus_ && i2c_bus_->RemoveDevice(device_index)) {
                // Remove from tracking vectors
                i2c_device_indices_.erase(index_iter);
                i2c_device_addresses_.erase(i2c_device_addresses_.begin() + vector_index);
                i2c_device_external_.erase(i2c_device_external_.begin() + vector_index);
                i2c_device_to_bus_mapping_.erase(device_index);
                
                Logger::GetInstance().Info("CommChannelsManager", "Removed external I2C device with index %d from bus 0", device_index);
                return true;
            } else {
                Logger::GetInstance().Error("CommChannelsManager", "Failed to remove I2C device at index %d from bus", device_index);
                return false;
            }
        } else {
            Logger::GetInstance().Error("CommChannelsManager", "Cannot remove built-in I2C device with index %d", device_index);
            return false;
        }
    }
    
    Logger::GetInstance().Error("CommChannelsManager", "I2C device with index %d not found", device_index);
    return false;
}

bool CommChannelsManager::HasI2cDeviceAtAddress(uint8_t bus_index, uint8_t device_address) const noexcept {
    if (bus_index == 0) {
        // Check built-in devices on Bus 0
        return std::find(i2c_device_addresses_.begin(), i2c_device_addresses_.end(), device_address) != i2c_device_addresses_.end();
    } else {
        // Check custom devices on external buses
        auto custom_bus_iter = custom_i2c_devices_.find(bus_index);
        if (custom_bus_iter != custom_i2c_devices_.end()) {
            // Note: Custom devices don't store addresses, so we can't check them here
            // This is a limitation of the current implementation
            // In a full implementation, we would need to store address mappings for custom devices
            return false;
        }
        return false;
    }
}







//==================== RUNTIME SPI DEVICE MANAGEMENT ====================//

int CommChannelsManager::RegisterCustomSpiDevice(std::shared_ptr<BaseSpi> custom_device, int device_index, uint8_t bus_index) noexcept {
    if (bus_index == 0xFF) {
        // Auto-assign to next available external bus
        bus_index = 10;  // Start with external bus 10
        while (external_spi_buses_.find(bus_index) != external_spi_buses_.end()) {
            bus_index++;
        }
    }
    
    if (!custom_device) {
        Logger::GetInstance().Error("CommChannelsManager", "Cannot register null SPI device");
        return -1;
    }
    
    // Auto-assign device index if not specified
    if (device_index == -1) {
        device_index = GetNextAvailableDeviceIndex(bus_index);
        if (device_index == -1) {
            Logger::GetInstance().Error("CommChannelsManager", "No available SPI device slots for bus %d", bus_index);
            return -1;
        }
    }
    
    // Register the external bus if it's new
    if (bus_index >= 10) {
        external_spi_buses_.insert(bus_index);
    }
    
    // Store the custom device
    custom_spi_devices_[device_index] = custom_device;
    spi_device_to_bus_mapping_[device_index] = bus_index;
    
    Logger::GetInstance().Info("CommChannelsManager", "Registered custom SPI device with index %d on bus %d", device_index, bus_index);
    
    return device_index;
}

bool CommChannelsManager::RemoveSpiDevice(int device_index) noexcept {
    if (device_index < 0) {
        Logger::GetInstance().Error("CommChannelsManager", "Invalid SPI device index: %d", device_index);
        return false;
    }
    
    // Check if this is a custom SPI device (external bus)
    auto custom_iter = custom_spi_devices_.find(device_index);
    if (custom_iter != custom_spi_devices_.end()) {
        // Get the bus index for this device
        auto mapping_iter = spi_device_to_bus_mapping_.find(device_index);
        if (mapping_iter != spi_device_to_bus_mapping_.end()) {
            uint8_t bus_index = mapping_iter->second;
            
            // Remove from custom devices
            custom_spi_devices_.erase(custom_iter);
            spi_device_to_bus_mapping_.erase(mapping_iter);
            
            Logger::GetInstance().Info("CommChannelsManager", "Removed custom SPI device with index %d from bus %d", device_index, bus_index);
            return true;
        }
    }
    
    // Check if this is a built-in device (Bus 0)
    auto index_iter = std::find(spi_device_indices_.begin(), spi_device_indices_.end(), device_index);
    if (index_iter != spi_device_indices_.end()) {
        Logger::GetInstance().Error("CommChannelsManager", "Cannot remove built-in SPI device with index %d", device_index);
        return false;
    }
    
    Logger::GetInstance().Error("CommChannelsManager", "SPI device with index %d not found", device_index);
    return false;
}





int CommChannelsManager::RegisterCustomI2cDevice(uint8_t bus_index, std::shared_ptr<BaseI2c> custom_device, uint8_t device_address) noexcept {
    if (!custom_device) {
        Logger::GetInstance().Error("CommChannelsManager", "Cannot register null I2C device");
        return -1;
    }
    
    // Get next available device index for this bus
    int device_index = GetNextAvailableDeviceIndex(bus_index);
    if (device_index == -1) {
        Logger::GetInstance().Error("CommChannelsManager", "No available I2C device slots for bus %d", bus_index);
        return -1;
    }
    
    // Register the external bus if it's new
    if (bus_index >= 10) {
        external_i2c_buses_.insert(bus_index);
    }
    
    // Store the custom device
    custom_i2c_devices_[device_index] = custom_device;
    i2c_device_to_bus_mapping_[device_index] = bus_index;
    
    Logger::GetInstance().Info("CommChannelsManager", "Registered custom I2C device with index %d on bus %d", device_index, bus_index);
    
    return device_index;
}

int CommChannelsManager::RegisterCustomI2cDevice(std::shared_ptr<BaseI2c> custom_device, uint8_t device_address, uint8_t bus_index) noexcept {
    if (bus_index == 0xFF) {
        // Auto-assign to next available external bus
        bus_index = 10;  // Start with external bus 10
        while (external_i2c_buses_.find(bus_index) != external_i2c_buses_.end()) {
            bus_index++;
        }
    }
    
    return RegisterCustomI2cDevice(bus_index, custom_device, device_address);
}

//==================== UART Accessors ====================//

bool CommChannelsManager::GetUart(std::size_t bus_index, BaseUart*& bus) noexcept {
    if (bus_index >= uart_buses_.size()) {
        // Return first available UART bus or return false if none available
        if (uart_buses_.empty()) {
            bus = nullptr;
            return false;
        }
        bus = uart_buses_[0].get();
        return true;
    }
    bus = uart_buses_[bus_index].get();
    return true;
}

std::size_t CommChannelsManager::GetUartCount() const noexcept {
    return uart_buses_.size();
}

//==================== CAN Accessors ====================//

bool CommChannelsManager::GetCan(std::size_t bus_index, BaseCan*& bus) noexcept {
    if (bus_index >= can_buses_.size()) {
        // Return first available CAN bus or return false if none available
        if (can_buses_.empty()) {
            bus = nullptr;
            return false;
        }
        bus = can_buses_[0].get();
        return true;
    }
    bus = can_buses_[bus_index].get();
    return true;
}

std::size_t CommChannelsManager::GetCanCount() const noexcept {
    return can_buses_.size();
} 

void CommChannelsManager::RegisterBuiltinDevices() noexcept {
    // Register all built-in SPI devices with Bus 0
    for (int device_index : spi_device_indices_) {
        spi_device_to_bus_mapping_[device_index] = 0;  // All built-in SPI devices on Bus 0
    }
    
    // Register all built-in I2C devices with Bus 0
    for (int device_index : i2c_device_indices_) {
        i2c_device_to_bus_mapping_[device_index] = 0;  // All built-in I2C devices on Bus 0
    }
    
    // Establish device ID to bus/device mapping for known devices
    // This mapping is established during initialization and remains fixed
    // SPI Device IDs map to Bus 0 with their enum values as device indices
    spi_device_to_bus_mapping_[static_cast<int>(SpiDeviceId::TMC9660_MOTOR_CONTROLLER)] = 0;
    spi_device_to_bus_mapping_[static_cast<int>(SpiDeviceId::AS5047U_POSITION_ENCODER)] = 0;
    spi_device_to_bus_mapping_[static_cast<int>(SpiDeviceId::EXTERNAL_DEVICE_1)] = 0;
    spi_device_to_bus_mapping_[static_cast<int>(SpiDeviceId::EXTERNAL_DEVICE_2)] = 0;
    
    // I2C Device IDs map to Bus 0 with their enum values as device indices
    i2c_device_to_bus_mapping_[static_cast<int>(I2cDeviceId::BNO08X_IMU)] = 0;
    i2c_device_to_bus_mapping_[static_cast<int>(I2cDeviceId::PCAL9555_GPIO_EXPANDER)] = 0;
    i2c_device_to_bus_mapping_[static_cast<int>(I2cDeviceId::EXTERNAL_DEVICE_1)] = 0;
    i2c_device_to_bus_mapping_[static_cast<int>(I2cDeviceId::EXTERNAL_DEVICE_2)] = 0;
    
    Logger::GetInstance().Info("CommChannelsManager", "Registered %zu SPI devices and %zu I2C devices on Bus 0", 
             spi_device_indices_.size(), i2c_device_indices_.size());
    Logger::GetInstance().Info("CommChannelsManager", "Established device ID to bus mapping for all known devices");
} 

//==================== BUS MANAGEMENT ====================//

uint8_t CommChannelsManager::GetBusCount() const noexcept {
    uint8_t count = 0;
    
    // Count built-in buses
    if (spi_bus_) count++;  // SPI Bus 0
    if (i2c_bus_) count++;  // I2C Bus 0
    
    // Count external buses
    count += external_spi_buses_.size();
    count += external_i2c_buses_.size();
    
    return count;
}

bool CommChannelsManager::IsBusAvailable(uint8_t bus_index) const noexcept {
    // Check built-in buses
    if (bus_index == 0) {
        return (spi_bus_ != nullptr || i2c_bus_ != nullptr);
    }
    
    // Check external buses
    if (bus_index >= 10) {
        return (external_spi_buses_.find(bus_index) != external_spi_buses_.end() ||
                external_i2c_buses_.find(bus_index) != external_i2c_buses_.end());
    }
    
    return false;
}

uint8_t CommChannelsManager::GetDeviceCountOnBus(uint8_t bus_index) const noexcept {
    uint8_t count = 0;
    
    // Count devices on built-in bus (Bus 0)
    if (bus_index == 0) {
        count += spi_device_indices_.size();  // Built-in SPI devices
        count += i2c_device_indices_.size();  // Built-in I2C devices
    }
    
    // Count devices on external buses (Bus 10+)
    if (bus_index >= 10) {
        // Count SPI devices on this external bus
        for (const auto& mapping : spi_device_to_bus_mapping_) {
            if (mapping.second == bus_index) {
                count++;
            }
        }
        
        // Count I2C devices on this external bus
        for (const auto& mapping : i2c_device_to_bus_mapping_) {
            if (mapping.second == bus_index) {
                count++;
            }
        }
    }
    
    return count;
}

bool CommChannelsManager::GetMcuI2cBus(uint8_t bus_index, EspI2cBus*& bus) noexcept {
    if (bus_index == 0 && i2c_bus_) {
        bus = i2c_bus_.get();
        return true;
    }
    // For now, only Bus 0 is supported for ESP32 I2C
    // In future revisions, this could support multiple ESP32 I2C buses
    bus = nullptr;
    return false;
}

bool CommChannelsManager::GetMcuSpiBus(uint8_t bus_index, EspSpiBus*& bus) noexcept {
    if (bus_index == 0 && spi_bus_) {
        bus = spi_bus_.get();
        return true;
    }
    // For now, only Bus 0 is supported for ESP32 SPI
    // In future revisions, this could support multiple ESP32 SPI buses
    bus = nullptr;
    return false;
} 

int CommChannelsManager::GetNextAvailableDeviceIndex(uint8_t bus_index) const noexcept {
    // For built-in bus (Bus 0), find next available index
    if (bus_index == 0) {
        // Start with index 0 and find the first available
        int index = 0;
        while (true) {
            // Check if this index is used by built-in devices
            bool used_by_builtin = (std::find(spi_device_indices_.begin(), spi_device_indices_.end(), index) != spi_device_indices_.end() ||
                                   std::find(i2c_device_indices_.begin(), i2c_device_indices_.end(), index) != i2c_device_indices_.end());
            
            // Check if this index is used by custom devices
            bool used_by_custom = (custom_spi_devices_.find(index) != custom_spi_devices_.end() ||
                                  custom_i2c_devices_.find(index) != custom_i2c_devices_.end());
            
            if (!used_by_builtin && !used_by_custom) {
                return index;
            }
            
            index++;
            
            // Prevent infinite loop (reasonable limit)
            if (index > 1000) {
                return -1;
            }
        }
    }
    
    // For external buses (Bus 10+), find next available index
    if (bus_index >= 10) {
        int index = 0;
        while (true) {
            // Check if this index is used by any device on this bus
            bool used_on_this_bus = false;
            
            // Check SPI devices on this bus
            auto spi_mapping = spi_device_to_bus_mapping_.find(index);
            if (spi_mapping != spi_device_to_bus_mapping_.end() && spi_mapping->second == bus_index) {
                used_on_this_bus = true;
            }
            
            // Check I2C devices on this bus
            auto i2c_mapping = i2c_device_to_bus_mapping_.find(index);
            if (i2c_mapping != i2c_device_to_bus_mapping_.end() && i2c_mapping->second == bus_index) {
                used_on_this_bus = true;
            }
            
            if (!used_on_this_bus) {
                return index;
            }
            
            index++;
            
            // Prevent infinite loop (reasonable limit)
            if (index > 1000) {
                return -1;
            }
        }
    }
    
    return -1;
}

void CommChannelsManager::DumpStatistics() const noexcept {
    static constexpr const char* TAG = "CommChannelsManager";
    
    Logger::GetInstance().Info(TAG, "=== COMM CHANNELS MANAGER STATISTICS ===");
    
    RtosMutex::LockGuard lock(mutex_);
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", initialized_.load() ? "YES" : "NO");
    
    // Bus Statistics
    Logger::GetInstance().Info(TAG, "Bus Configuration:");
    Logger::GetInstance().Info(TAG, "  SPI Bus: %s", spi_bus_ ? "ACTIVE" : "INACTIVE");
    Logger::GetInstance().Info(TAG, "  I2C Bus: %s", i2c_bus_ ? "ACTIVE" : "INACTIVE");
    Logger::GetInstance().Info(TAG, "  UART Buses: %d", static_cast<int>(uart_buses_.size()));
    Logger::GetInstance().Info(TAG, "  CAN Buses: %d", static_cast<int>(can_buses_.size()));
    
    // SPI Device Statistics
    Logger::GetInstance().Info(TAG, "SPI Device Statistics:");
    Logger::GetInstance().Info(TAG, "  Built-in SPI Devices: %d", static_cast<int>(spi_device_indices_.size()));
    Logger::GetInstance().Info(TAG, "  Custom SPI Devices: %d", static_cast<int>(custom_spi_devices_.size()));
    Logger::GetInstance().Info(TAG, "  External SPI Buses: %d", static_cast<int>(external_spi_buses_.size()));
    
    if (!spi_device_indices_.empty()) {
        Logger::GetInstance().Info(TAG, "  Built-in SPI Device IDs:");
        for (size_t i = 0; i < spi_device_indices_.size() && i < 8; ++i) {
            SpiDeviceId device_id = static_cast<SpiDeviceId>(i);
            const char* device_name = "Unknown";
            switch (device_id) {
                case SpiDeviceId::TMC9660_MOTOR_CONTROLLER: device_name = "TMC9660 Motor Controller"; break;
                case SpiDeviceId::AS5047U_POSITION_ENCODER: device_name = "AS5047U Position Encoder"; break;
                case SpiDeviceId::EXTERNAL_DEVICE_1: device_name = "External Device 1"; break;
                case SpiDeviceId::EXTERNAL_DEVICE_2: device_name = "External Device 2"; break;
                default: device_name = "Unknown Device"; break;
            }
            Logger::GetInstance().Info(TAG, "    [%d] %s (Index: %d)", 
                static_cast<int>(i), device_name, spi_device_indices_[i]);
        }
    }
    
    // I2C Device Statistics
    Logger::GetInstance().Info(TAG, "I2C Device Statistics:");
    Logger::GetInstance().Info(TAG, "  Built-in I2C Devices: %d", static_cast<int>(i2c_device_indices_.size()));
    Logger::GetInstance().Info(TAG, "  Custom I2C Devices: %d", static_cast<int>(custom_i2c_devices_.size()));
    Logger::GetInstance().Info(TAG, "  External I2C Buses: %d", static_cast<int>(external_i2c_buses_.size()));
    
    if (!i2c_device_indices_.empty()) {
        Logger::GetInstance().Info(TAG, "  Built-in I2C Device IDs:");
        for (size_t i = 0; i < i2c_device_indices_.size() && i < 8; ++i) {
            I2cDeviceId device_id = static_cast<I2cDeviceId>(i);
            const char* device_name = "Unknown";
            uint8_t address = (i < i2c_device_addresses_.size()) ? i2c_device_addresses_[i] : 0xFF;
            
            switch (device_id) {
                case I2cDeviceId::BNO08X_IMU: device_name = "BNO08x IMU"; break;
                case I2cDeviceId::PCAL9555_GPIO_EXPANDER: device_name = "PCAL9555 GPIO Expander"; break;
                default: device_name = "Unknown Device"; break;
            }
            Logger::GetInstance().Info(TAG, "    [%d] %s (Index: %d, Addr: 0x%02X)", 
                static_cast<int>(i), device_name, i2c_device_indices_[i], address);
        }
    }
    
    // Device-to-Bus Mapping
    Logger::GetInstance().Info(TAG, "Device-to-Bus Mapping:");
    if (!spi_device_to_bus_mapping_.empty()) {
        Logger::GetInstance().Info(TAG, "  SPI Device-Bus Mappings:");
        for (const auto& mapping : spi_device_to_bus_mapping_) {
            Logger::GetInstance().Info(TAG, "    Device %d -> Bus %d", mapping.first, mapping.second);
        }
    }
    
    if (!i2c_device_to_bus_mapping_.empty()) {
        Logger::GetInstance().Info(TAG, "  I2C Device-Bus Mappings:");
        for (const auto& mapping : i2c_device_to_bus_mapping_) {
            Logger::GetInstance().Info(TAG, "    Device %d -> Bus %d", mapping.first, mapping.second);
        }
    }
    
    // UART/CAN Statistics
    Logger::GetInstance().Info(TAG, "UART/CAN Statistics:");
    Logger::GetInstance().Info(TAG, "  Total UART Count: %d", GetUartCount());
    Logger::GetInstance().Info(TAG, "  Total CAN Count: %d", GetCanCount());
    
    // Overall Statistics
    int total_devices = static_cast<int>(spi_device_indices_.size() + custom_spi_devices_.size() + 
                                       i2c_device_indices_.size() + custom_i2c_devices_.size());
    int total_buses = static_cast<int>(uart_buses_.size() + can_buses_.size() + 
                                     external_spi_buses_.size() + external_i2c_buses_.size());
    if (spi_bus_) total_buses++;
    if (i2c_bus_) total_buses++;
    
    Logger::GetInstance().Info(TAG, "Overall Summary:");
    Logger::GetInstance().Info(TAG, "  Total Devices Managed: %d", total_devices);
    Logger::GetInstance().Info(TAG, "  Total Buses Managed: %d", total_buses);
    Logger::GetInstance().Info(TAG, "  System Status: %s", initialized_.load() ? "OPERATIONAL" : "NOT_READY");
    
    Logger::GetInstance().Info(TAG, "=== END COMM CHANNELS MANAGER STATISTICS ===");
} 
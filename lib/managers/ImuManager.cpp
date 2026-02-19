#include "ImuManager.h"

// Bno08x handler for unified IMU interface
#include "handlers/bno08x/Bno08xHandler.h"

// Communication manager for I2C/SPI access
#include "CommChannelsManager.h"

// GPIO manager for interrupt pin access
#include "GpioManager.h"

// Logger for unified logging
#include "handlers/logger/Logger.h"

// Base interfaces
#include "base/BaseI2c.h"
#include "base/BaseSpi.h"
#include "base/BaseGpio.h"

// RtosMutex for thread safety
#include "RtosMutex.h"

// Standard library
#include <iostream>
#include <atomic>

// ESP-IDF for semaphores
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
}

//==============================================================================
// IMUMANAGER IMPLEMENTATION
//==============================================================================

ImuManager& ImuManager::GetInstance() noexcept {
    static ImuManager instance;
    return instance;
}

ImuManager::ImuManager() noexcept 
    : initialized_(false), manager_mutex_(), onboard_device_created_(false) {
    // Initialize all device slots as empty and not active
    for (auto& h : bno08x_handlers_) h.reset();
    device_initialized_.fill(false);
    device_active_.fill(false);
    i2c_device_indices_.fill(-1); // Initialize to -1 (no I2C device)
}

ImuManager::~ImuManager() noexcept {
    Deinitialize();
}

bool ImuManager::EnsureInitialized() noexcept {
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    MutexLockGuard lock(manager_mutex_);
    
    // Double-check after acquiring lock
    if (initialized_.load(std::memory_order_acquire)) {
        return true;
    }
    
    return Initialize();
}

bool ImuManager::IsInitialized() const noexcept {
    return initialized_.load(std::memory_order_acquire);
}

bool ImuManager::Deinitialize() noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (!initialized_.load(std::memory_order_acquire)) {
        return true;
    }

         Logger::GetInstance().Info("ImuManager", "Deinitializing IMU Manager");

    // Disable interrupt first
    if (interrupt_enabled_) {
        DisableInterrupt(ONBOARD_IMU_INDEX);
    }

    // Clean up interrupt semaphore
    if (interrupt_semaphore_) {
        vSemaphoreDelete(static_cast<SemaphoreHandle_t>(interrupt_semaphore_));
        interrupt_semaphore_ = nullptr;
    }

    // Reset interrupt state
    interrupt_gpio_ = nullptr;
    interrupt_gpio_shared_.reset();  // Release shared ownership
    interrupt_configured_ = false;
    interrupt_enabled_ = false;
    interrupt_count_.store(0);
    interrupt_callback_ = nullptr;

    // Deinitialize all BNO08x handlers and clean up I2C devices
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        if (device_active_[i] && bno08x_handlers_[i]) {
            // Clean up I2C device if this slot had one
            if (i2c_device_indices_[i] >= 0 && comm_manager_) {
                comm_manager_->RemoveI2cDevice(i2c_device_indices_[i]);
                                 Logger::GetInstance().Info("ImuManager", "Cleaned up I2C device index %d for slot %u", i2c_device_indices_[i], i);
            }
            
            bno08x_handlers_[i].reset();
            device_active_[i] = false;
            device_initialized_[i] = false;
            i2c_device_indices_[i] = -1;
                         Logger::GetInstance().Info("ImuManager", "BNO08x handler %u deinitialized", i);
        }
    }

    initialized_.store(false, std::memory_order_release);
    comm_manager_ = nullptr;
    gpio_manager_ = nullptr;
    onboard_device_created_ = false;

         Logger::GetInstance().Info("ImuManager", "IMU Manager deinitialized");
    return true;
}

//**************************************************************************//
//**                  HANDLER AND DRIVER MANAGEMENT                       **//
//**************************************************************************//

Bno08xHandler* ImuManager::GetBno08xHandler(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (deviceIndex >= MAX_IMU_DEVICES || !device_active_[deviceIndex] || !bno08x_handlers_[deviceIndex]) {
        return nullptr;
    }
    
    return bno08x_handlers_[deviceIndex].get();
}

IBno08xDriverOps* ImuManager::GetSensor(uint8_t deviceIndex) noexcept {
    Bno08xHandler* handler = GetBno08xHandler(deviceIndex);
    return handler ? handler->GetSensor() : nullptr;
}

//**************************************************************************//
//**                  DEVICES MANAGEMENT METHODS                           **//
//**************************************************************************//

bool ImuManager::CreateExternalBno08xDevice(uint8_t deviceIndex, 
                                           uint8_t i2c_address,
                                           uint32_t i2c_speed_hz,
                                           const Bno08xConfig& config) {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("ImuManager", "Invalid device index %u for external device", deviceIndex);
        return false;
    }
    
    if (device_active_[deviceIndex]) {
        Logger::GetInstance().Error("ImuManager", "Device slot %u already occupied", deviceIndex);
        return false;
    }
    
    // Validate I2C address (7-bit addressing: 0x08-0x77)
    if (i2c_address < 0x08 || i2c_address > 0x77) {
        Logger::GetInstance().Error("ImuManager", "Invalid I2C address: 0x%02X (must be 0x08-0x77)", i2c_address);
        return false;
    }
    
    // Get CommChannelsManager instance
    if (!comm_manager_ || !comm_manager_->IsInitialized()) {
        Logger::GetInstance().Error("ImuManager", "CommChannelsManager not initialized");
        return false;
    }
    
    // Check if I2C device already exists at this address on Bus 0
    if (comm_manager_->HasI2cDeviceAtAddress(0, i2c_address)) {
        Logger::GetInstance().Error("ImuManager", "I2C device already exists at address 0x%02X on Bus 0", i2c_address);
        return false;
    }
    
    // Create I2C device at runtime
    int i2c_device_index = comm_manager_->CreateI2cDevice(i2c_address, i2c_speed_hz);
    if (i2c_device_index < 0) {
        Logger::GetInstance().Error("ImuManager", "Failed to create I2C device at address 0x%02X", i2c_address);
        return false;
    }
    
    // Get the I2C interface for the newly created device (on Bus 0)
    BaseI2c* i2c_interface = comm_manager_->GetI2cDevice(0, i2c_device_index);
    if (!i2c_interface) {
        Logger::GetInstance().Error("ImuManager", "Failed to get I2C interface for device index %d", i2c_device_index);
        // Clean up the I2C device
        comm_manager_->RemoveI2cDevice(i2c_device_index);
        return false;
    }
    
    // Create external BNO08x handler
    auto external_handler = std::make_unique<Bno08xHandler>(*i2c_interface, config);
    if (!external_handler) {
        Logger::GetInstance().Error("ImuManager", "Failed to create BNO08x handler for device %u", deviceIndex);
        // Clean up the I2C device
        comm_manager_->RemoveI2cDevice(i2c_device_index);
        return false;
    }
    
    // Store the handler and track the I2C device index
    bno08x_handlers_[deviceIndex] = std::move(external_handler);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = false;
    i2c_device_indices_[deviceIndex] = i2c_device_index; // Track for cleanup
    
    Logger::GetInstance().Info("ImuManager", "External BNO08x device %u created successfully: I2C address=0x%02X, speed=%uHz", 
             deviceIndex, i2c_address, i2c_speed_hz);
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        device_initialized_[deviceIndex] = bno08x_handlers_[deviceIndex]->Initialize() == Bno08xError::SUCCESS;
        if (device_initialized_[deviceIndex]) {
            Logger::GetInstance().Info("ImuManager", "External BNO08x device %u initialized successfully", deviceIndex);
        } else {
            Logger::GetInstance().Error("ImuManager", "Failed to initialize external BNO08x device %u", deviceIndex);
        }
    }
    
    return true;
}



bool ImuManager::CreateExternalBno08xDevice(uint8_t deviceIndex, 
                                           BaseI2c& i2c_interface,
                                           const Bno08xConfig& config) {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("ImuManager", "Invalid device index %u for external device", deviceIndex);
        return false;
    }
    
    if (device_active_[deviceIndex]) {
        Logger::GetInstance().Error("ImuManager", "Device slot %u already occupied", deviceIndex);
        return false;
    }
    
    // Create external BNO08x handler with provided I2C interface
    auto external_handler = std::make_unique<Bno08xHandler>(i2c_interface, config);
    if (!external_handler) {
        Logger::GetInstance().Error("ImuManager", "Failed to create BNO08x handler for device %u", deviceIndex);
        return false;
    }
    
    bno08x_handlers_[deviceIndex] = std::move(external_handler);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = false;
    
    Logger::GetInstance().Info("ImuManager", "External BNO08x device %u created successfully (BaseI2c interface)", deviceIndex);
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        device_initialized_[deviceIndex] = bno08x_handlers_[deviceIndex]->Initialize() == Bno08xError::SUCCESS;
        if (device_initialized_[deviceIndex]) {
            Logger::GetInstance().Info("ImuManager", "External BNO08x device %u initialized successfully", deviceIndex);
        } else {
            Logger::GetInstance().Error("ImuManager", "Failed to initialize external BNO08x device %u", deviceIndex);
        }
    }
    
    return true;
}

bool ImuManager::CreateExternalBno08xDevice(uint8_t deviceIndex, 
                                           SpiDeviceId spiDeviceId,
                                           const Bno08xConfig& config) {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("ImuManager", "Invalid device index %u for external device", deviceIndex);
        return false;
    }
    
    if (device_active_[deviceIndex]) {
        Logger::GetInstance().Error("ImuManager", "Device slot %u already occupied", deviceIndex);
        return false;
    }
    
    // Get SPI interface from CommChannelsManager
    if (!comm_manager_ || !comm_manager_->IsInitialized()) {
        Logger::GetInstance().Error("ImuManager", "CommChannelsManager not initialized");
        return false;
    }
    
    BaseSpi* spi_interface = comm_manager_->GetSpiDevice(spiDeviceId);
    if (!spi_interface) {
        Logger::GetInstance().Error("ImuManager", "SPI device %u not available", static_cast<uint8_t>(spiDeviceId));
        return false;
    }
    
    // Create external BNO08x handler with SPI interface
    auto external_handler = std::make_unique<Bno08xHandler>(*spi_interface, config);
    if (!external_handler) {
        Logger::GetInstance().Error("ImuManager", "Failed to create BNO08x handler for device %u", deviceIndex);
        return false;
    }
    
    bno08x_handlers_[deviceIndex] = std::move(external_handler);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = false;
    
    Logger::GetInstance().Info("ImuManager", "External BNO08x device %u created successfully (SPI)", deviceIndex);
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        device_initialized_[deviceIndex] = bno08x_handlers_[deviceIndex]->Initialize() == Bno08xError::SUCCESS;
        if (device_initialized_[deviceIndex]) {
            Logger::GetInstance().Info("ImuManager", "External BNO08x device %u initialized successfully", deviceIndex);
        } else {
            Logger::GetInstance().Error("ImuManager", "Failed to initialize external BNO08x device %u", deviceIndex);
        }
    }
    
    return true;
}

bool ImuManager::CreateExternalBno08xDevice(uint8_t deviceIndex, 
                                           BaseSpi& spi_interface,
                                           const Bno08xConfig& config) {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("ImuManager", "Invalid device index %u for external device", deviceIndex);
        return false;
    }
    
    if (device_active_[deviceIndex]) {
        Logger::GetInstance().Error("ImuManager", "Device slot %u already occupied", deviceIndex);
        return false;
    }
    
    // Create external BNO08x handler with provided SPI interface
    auto external_handler = std::make_unique<Bno08xHandler>(spi_interface, config);
    if (!external_handler) {
        Logger::GetInstance().Error("ImuManager", "Failed to create BNO08x handler for device %u", deviceIndex);
        return false;
    }
    
    bno08x_handlers_[deviceIndex] = std::move(external_handler);
    device_active_[deviceIndex] = true;
    device_initialized_[deviceIndex] = false;
    
    Logger::GetInstance().Info("ImuManager", "External BNO08x device %u created successfully (BaseSpi interface)", deviceIndex);
    
    // If system is already initialized, initialize this device immediately
    if (IsInitialized()) {
        device_initialized_[deviceIndex] = bno08x_handlers_[deviceIndex]->Initialize() == Bno08xError::SUCCESS;
        if (device_initialized_[deviceIndex]) {
            Logger::GetInstance().Info("ImuManager", "External BNO08x device %u initialized successfully", deviceIndex);
        } else {
            Logger::GetInstance().Error("ImuManager", "Failed to initialize external BNO08x device %u", deviceIndex);
        }
    }
    
    return true;
}

bool ImuManager::DeleteExternalDevice(uint8_t deviceIndex) {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        Logger::GetInstance().Error("ImuManager", "Cannot delete onboard device or invalid index %u", deviceIndex);
        return false;
    }
    
    if (!device_active_[deviceIndex]) {
        Logger::GetInstance().Error("ImuManager", "Device %u doesn't exist", deviceIndex);
        return false;
    }
    
    // Get the I2C device index associated with the handler
    int i2c_device_index = i2c_device_indices_[deviceIndex];

    // Clean up the BNO08x handler
    bno08x_handlers_[deviceIndex].reset();
    device_active_[deviceIndex] = false;
    device_initialized_[deviceIndex] = false;
    i2c_device_indices_[deviceIndex] = -1; // Mark for removal

    // Clean up the I2C device from CommChannelsManager
    if (comm_manager_ && i2c_device_index >= 0) {
        comm_manager_->RemoveI2cDevice(i2c_device_index);
        Logger::GetInstance().Info("ImuManager", "I2C device index %d removed from CommChannelsManager", i2c_device_index);
    }

    Logger::GetInstance().Info("ImuManager", "External IMU device %u deleted successfully", deviceIndex);
    return true;
}

bool ImuManager::IsExternalDeviceIndex(uint8_t deviceIndex) const noexcept {
    return (deviceIndex == EXTERNAL_IMU_1_INDEX || 
            deviceIndex == EXTERNAL_IMU_2_INDEX || 
            deviceIndex == EXTERNAL_IMU_3_INDEX);
}

uint8_t ImuManager::GetDeviceCount() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        if (device_active_[i]) {
            count++;
        }
    }
    return count;
}

bool ImuManager::IsDeviceValid(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    return (deviceIndex < MAX_IMU_DEVICES) && device_active_[deviceIndex];
}

bool ImuManager::IsExternalSlotAvailable(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (!IsExternalDeviceIndex(deviceIndex)) {
        return false;
    }
    
    return !device_active_[deviceIndex];
}

std::vector<uint8_t> ImuManager::GetActiveDeviceIndices() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    std::vector<uint8_t> activeIndices;
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        if (device_active_[i]) {
            activeIndices.push_back(i);
        }
    }
    return activeIndices;
}

std::vector<bool> ImuManager::InitializeAllDevices() {
    MutexLockGuard lock(manager_mutex_);
    
    std::vector<bool> results;
    
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        if (device_active_[i] && bno08x_handlers_[i]) {
            device_initialized_[i] = bno08x_handlers_[i]->Initialize() == Bno08xError::SUCCESS;
            results.push_back(device_initialized_[i]);
            
            if (device_initialized_[i]) {
                Logger::GetInstance().Info("ImuManager", "BNO08x device %u initialized successfully", i);
            } else {
                Logger::GetInstance().Error("ImuManager", "Failed to initialize BNO08x device %u", i);
            }
        }
    }
    
    return results;
}

std::vector<bool> ImuManager::GetInitializationStatus() const {
    MutexLockGuard lock(manager_mutex_);
    
    std::vector<bool> status;
    
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        if (device_active_[i]) {
            status.push_back(device_initialized_[i]);
        }
    }
    
    return status;
}

//**************************************************************************//
//**                  DEVICE INFORMATION METHODS                          **//
//**************************************************************************//

std::vector<std::string> ImuManager::GetAvailableDevices() const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    std::vector<std::string> devices;
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        if (device_active_[i]) {
            std::string device_name = "BNO08x Device " + std::to_string(i);
            if (i == ONBOARD_IMU_INDEX) {
                device_name += " (Onboard)";
            } else {
                device_name += " (External)";
            }
            devices.push_back(device_name);
        }
    }
    return devices;
}

std::string ImuManager::GetDeviceType(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (deviceIndex >= MAX_IMU_DEVICES || !device_active_[deviceIndex]) {
        return "Unknown";
    }
    
    return "BNO08x";
}

//**************************************************************************//
//**                  INTERRUPT SUPPORT METHODS                           **//
//**************************************************************************//

bool ImuManager::ConfigureInterrupt(uint8_t deviceIndex, std::function<void()> callback) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    if (!initialized_.load(std::memory_order_acquire)) {
        Logger::GetInstance().Error("ImuManager", "ImuManager not initialized");
        return false;
    }

    // For now, only support interrupt configuration for onboard device
    if (deviceIndex != ONBOARD_IMU_INDEX) {
        Logger::GetInstance().Error("ImuManager", "Interrupt configuration only supported for onboard device (index 0)");
        return false;
    }

    if (!interrupt_gpio_) {
        Logger::GetInstance().Error("ImuManager", "Interrupt GPIO not available");
        return false;
    }

    if (interrupt_configured_) {
        Logger::GetInstance().Warn("ImuManager", "Interrupt already configured - reconfiguring");
        // Disable first if enabled
        if (interrupt_enabled_) {
            DisableInterrupt(deviceIndex);
        }
    }

    // Store user callback
    interrupt_callback_ = callback;

    // Configure GPIO interrupt (falling edge for active-low BNO08x INT)
    auto configure_result = interrupt_gpio_->ConfigureInterrupt(
        hf_gpio_interrupt_trigger_t::HF_GPIO_INTERRUPT_TRIGGER_FALLING_EDGE,
        GpioInterruptHandler,
        this  // Pass ImuManager instance as user data
    );

    if (configure_result != hf_gpio_err_t::GPIO_SUCCESS) {
        Logger::GetInstance().Error("ImuManager", "Failed to configure GPIO interrupt: %d", static_cast<int>(configure_result));
        interrupt_callback_ = nullptr;
        return false;
    }

    interrupt_configured_ = true;
    Logger::GetInstance().Info("ImuManager", "BNO08x interrupt configured successfully for device %u", deviceIndex);
    return true;
}

bool ImuManager::EnableInterrupt(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    // For now, only support interrupt for onboard device
    if (deviceIndex != ONBOARD_IMU_INDEX) {
        Logger::GetInstance().Error("ImuManager", "Interrupt only supported for onboard device (index 0)");
        return false;
    }
    
    if (!interrupt_configured_) {
        Logger::GetInstance().Error("ImuManager", "Interrupt not configured - call ConfigureInterrupt() first");
        return false;
    }

    if (interrupt_enabled_) {
        Logger::GetInstance().Warn("ImuManager", "Interrupt already enabled");
        return true;
    }

    auto enable_result = interrupt_gpio_->EnableInterrupt();
    if (enable_result != hf_gpio_err_t::GPIO_SUCCESS) {
        Logger::GetInstance().Error("ImuManager", "Failed to enable GPIO interrupt: %d", static_cast<int>(enable_result));
        return false;
    }

    interrupt_enabled_ = true;
    Logger::GetInstance().Info("ImuManager", "BNO08x interrupt enabled successfully for device %u", deviceIndex);
    return true;
}

bool ImuManager::DisableInterrupt(uint8_t deviceIndex) noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    // For now, only support interrupt for onboard device
    if (deviceIndex != ONBOARD_IMU_INDEX) {
        Logger::GetInstance().Error("ImuManager", "Interrupt only supported for onboard device (index 0)");
        return false;
    }
    
    if (!interrupt_enabled_) {
        return true;  // Already disabled
    }

    if (!interrupt_gpio_) {
        Logger::GetInstance().Error("ImuManager", "Interrupt GPIO not available");
        return false;
    }

    auto disable_result = interrupt_gpio_->DisableInterrupt();
    if (disable_result != hf_gpio_err_t::GPIO_SUCCESS) {
        Logger::GetInstance().Error("ImuManager", "Failed to disable GPIO interrupt: %d", static_cast<int>(disable_result));
        return false;
    }

    interrupt_enabled_ = false;
    Logger::GetInstance().Info("ImuManager", "BNO08x interrupt disabled successfully for device %u", deviceIndex);
    return true;
}

bool ImuManager::IsInterruptEnabled(uint8_t deviceIndex) const noexcept {
    MutexLockGuard lock(manager_mutex_);
    
    // For now, only support interrupt for onboard device
    if (deviceIndex != ONBOARD_IMU_INDEX) {
        return false;
    }
    
    return interrupt_enabled_;
}

bool ImuManager::WaitForInterrupt(uint8_t deviceIndex, uint32_t timeout_ms) noexcept {
    // For now, only support interrupt for onboard device
    if (deviceIndex != ONBOARD_IMU_INDEX) {
        Logger::GetInstance().Error("ImuManager", "Interrupt only supported for onboard device (index 0)");
        return false;
    }
    
    if (!interrupt_semaphore_) {
        Logger::GetInstance().Error("ImuManager", "Interrupt semaphore not available");
        return false;
    }

    if (!interrupt_enabled_) {
        Logger::GetInstance().Error("ImuManager", "Interrupt not enabled");
        return false;
    }

    TickType_t timeout_ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    BaseType_t result = xSemaphoreTake(static_cast<SemaphoreHandle_t>(interrupt_semaphore_), timeout_ticks);
    
    return (result == pdTRUE);
}

uint32_t ImuManager::GetInterruptCount(uint8_t deviceIndex) const noexcept {
    // For now, only support interrupt for onboard device
    if (deviceIndex != ONBOARD_IMU_INDEX) {
        return 0;
    }
    
    return interrupt_count_.load();
}

//==============================================================================
// PRIVATE IMPLEMENTATION
//==============================================================================

bool ImuManager::Initialize() noexcept {
    Logger::GetInstance().Info("ImuManager", "Initializing IMU Manager with multiple device support");

    // Get reference to communication manager
    comm_manager_ = &CommChannelsManager::GetInstance();
    if (!comm_manager_->EnsureInitialized()) {
        Logger::GetInstance().Error("ImuManager", "Failed to initialize CommChannelsManager");
        return false;
    }

    // Get reference to GPIO manager for interrupt support
    gpio_manager_ = &GpioManager::GetInstance();
    if (!gpio_manager_->IsInitialized()) {
        // Try to initialize GPIO manager
        bool init_result = gpio_manager_->EnsureInitialized();
        if (!init_result) {
            Logger::GetInstance().Warn("ImuManager", "GPIO Manager initialization failed - interrupt support will be limited");
        }
    }

    // Initialize onboard BNO08x IMU device
    bool onboard_ok = InitializeOnboardBno08xDevice();
    if (onboard_ok) {
        Logger::GetInstance().Info("ImuManager", "Onboard BNO08x IMU device successfully initialized");
    } else {
        Logger::GetInstance().Warn("ImuManager", "Onboard BNO08x IMU device initialization failed");
    }

    // Initialize interrupt GPIO (optional - don't fail if not available)
    if (gpio_manager_ && gpio_manager_->IsInitialized()) {
        bool gpio_ok = InitializeInterruptGpio();
        if (gpio_ok) {
            Logger::GetInstance().Info("ImuManager", "BNO08x interrupt GPIO initialized successfully");
        } else {
            Logger::GetInstance().Warn("ImuManager", "BNO08x interrupt GPIO not available - using polling mode only");
        }
    }

    // Consider initialized if at least onboard device is available
    initialized_.store(onboard_ok, std::memory_order_release);

    if (initialized_.load(std::memory_order_acquire)) {
        Logger::GetInstance().Info("ImuManager", "IMU Manager initialized successfully (%u devices available)", GetDeviceCount());
    } else {
        Logger::GetInstance().Error("ImuManager", "IMU Manager initialization failed - no devices available");
    }

    return initialized_.load(std::memory_order_acquire);
}

bool ImuManager::InitializeOnboardBno08xDevice() noexcept {
    Logger::GetInstance().Info("ImuManager", "Initializing onboard BNO08x IMU device with I2C transport");

    // Get I2C device from communication manager for BNO08x
    BaseI2c* imu_device = comm_manager_->GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    if (!imu_device) {
        Logger::GetInstance().Error("ImuManager", "BNO08x IMU device not available in CommChannelsManager");
        return false;
    }

    // Create onboard BNO08x handler with I2C interface
    Bno08xConfig config = Bno08xHandler::GetDefaultConfig();  // Use default configuration
    
    // Create the handler with I2C interface
    bno08x_handlers_[ONBOARD_IMU_INDEX] = std::make_unique<Bno08xHandler>(*imu_device, config);
    
    if (!bno08x_handlers_[ONBOARD_IMU_INDEX]) {
        Logger::GetInstance().Error("ImuManager", "Failed to create onboard BNO08x handler");
        return false;
    }
    
    // Initialize the handler
    Bno08xError init_result = bno08x_handlers_[ONBOARD_IMU_INDEX]->Initialize();
    if (init_result != Bno08xError::SUCCESS) {
        Logger::GetInstance().Error("ImuManager", "Failed to initialize onboard BNO08x handler: %s", Bno08xErrorToString(init_result));
        bno08x_handlers_[ONBOARD_IMU_INDEX].reset();
        return false;
    }
    
    device_active_[ONBOARD_IMU_INDEX] = true;
    device_initialized_[ONBOARD_IMU_INDEX] = true;
    onboard_device_created_ = true;
    
    Logger::GetInstance().Info("ImuManager", "Onboard BNO08x IMU device initialized successfully");
    return true;
}

//==============================================================================
// GPIO INTERRUPT SUPPORT IMPLEMENTATION
//==============================================================================

bool ImuManager::InitializeInterruptGpio() noexcept {
    if (!gpio_manager_) {
        Logger::GetInstance().Warn("ImuManager", "GPIO Manager not available for interrupt setup");
        return false;
    }

    // Get shared GPIO pin for PCAL_IMU_INT (creates if needed)
    // Configure as input with pull-up since BNO08x INT is active low
    interrupt_gpio_shared_ = gpio_manager_->Get("PCAL_IMU_INT");
    if (!interrupt_gpio_shared_) {
        Logger::GetInstance().Warn("ImuManager", "PCAL_IMU_INT functional pin not available on this platform");
        return false;
    }
    
    if (!interrupt_gpio_shared_) {
        Logger::GetInstance().Error("ImuManager", "Failed to create shared PCAL_IMU_INT pin");
        return false;
    }

    // Store raw pointer for compatibility with existing callback code
    interrupt_gpio_ = interrupt_gpio_shared_.get();

    // Create semaphore for WaitForInterrupt()
    interrupt_semaphore_ = xSemaphoreCreateBinary();
    if (!interrupt_semaphore_) {
        Logger::GetInstance().Error("ImuManager", "Failed to create interrupt semaphore");
        interrupt_gpio_ = nullptr;
        interrupt_gpio_shared_.reset();
        return false;
    }

    Logger::GetInstance().Info("ImuManager", "BNO08x interrupt GPIO (PCAL_IMU_INT) initialized successfully with shared pin access");
    return true;
}

void ImuManager::GpioInterruptHandler(BaseGpio* gpio, hf_gpio_interrupt_trigger_t trigger, void* user_data) noexcept {
    // This function executes in interrupt context - keep it minimal!
    auto* imu_manager = static_cast<ImuManager*>(user_data);
    if (!imu_manager) {
        return;
    }

    // Increment interrupt counter
    imu_manager->interrupt_count_.fetch_add(1);

    // Signal semaphore for WaitForInterrupt()
    if (imu_manager->interrupt_semaphore_) {
        BaseType_t higher_priority_task_woken = pdFALSE;
        xSemaphoreGiveFromISR(
            static_cast<SemaphoreHandle_t>(imu_manager->interrupt_semaphore_), 
            &higher_priority_task_woken
        );
        portYIELD_FROM_ISR(higher_priority_task_woken);
    }

    // Call user callback if provided
    if (imu_manager->interrupt_callback_) {
        imu_manager->interrupt_callback_();
    }
}

void ImuManager::DumpStatistics() const noexcept {
    static constexpr const char* TAG = "ImuManager";
    
    Logger::GetInstance().Info(TAG, "=== IMU MANAGER STATISTICS ===");
    
    MutexLockGuard lock(manager_mutex_);
    
    // System Health
    Logger::GetInstance().Info(TAG, "System Health:");
    Logger::GetInstance().Info(TAG, "  Initialized: %s", initialized_ ? "YES" : "NO");
    Logger::GetInstance().Info(TAG, "  Onboard Device Created: %s", onboard_device_created_ ? "YES" : "NO");
    
    // Device Statistics
    int active_devices = 0;
    int initialized_devices = 0;
    
    Logger::GetInstance().Info(TAG, "Device Status Summary:");
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        if (bno08x_handlers_[i] != nullptr) {
            bool is_active = device_active_[i];
            bool is_initialized = device_initialized_[i];
            uint32_t interrupt_count = GetInterruptCount(i);
            
            const char* device_type = (i == 0) ? "Onboard" : "External";
            Logger::GetInstance().Info(TAG, "  Device %d (%s): %s, %s, Interrupts: %d", 
                i, device_type,
                is_active ? "ACTIVE" : "INACTIVE",
                is_initialized ? "INITIALIZED" : "NOT_INITIALIZED",
                interrupt_count);
            
            if (is_active) active_devices++;
            if (is_initialized) initialized_devices++;
        } else {
            Logger::GetInstance().Info(TAG, "  Device %d: NOT_CREATED", i);
        }
    }
    
    // Overall Statistics
    Logger::GetInstance().Info(TAG, "Overall Statistics:");
    Logger::GetInstance().Info(TAG, "  Total Devices: %d", active_devices);
    Logger::GetInstance().Info(TAG, "  Initialized Devices: %d", initialized_devices);
    Logger::GetInstance().Info(TAG, "  Active Devices: %d", active_devices);
    
    // Interrupt Statistics
    uint32_t total_interrupts = 0;
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        total_interrupts += GetInterruptCount(i);
    }
    Logger::GetInstance().Info(TAG, "  Total Interrupts Processed: %d", total_interrupts);
    
    // Interrupt Configuration
    Logger::GetInstance().Info(TAG, "Interrupt Configuration:");
    Logger::GetInstance().Info(TAG, "  Interrupt Semaphore: %s", interrupt_semaphore_ ? "CONFIGURED" : "NOT_CONFIGURED");
    Logger::GetInstance().Info(TAG, "  Interrupt Callback: %s", interrupt_callback_ ? "SET" : "NOT_SET");
    
    // Memory Usage
    Logger::GetInstance().Info(TAG, "Memory Usage:");
    size_t handler_memory = 0;
    for (uint8_t i = 0; i < MAX_IMU_DEVICES; ++i) {
        if (bno08x_handlers_[i] != nullptr) {
            handler_memory += sizeof(Bno08xHandler);
        }
    }
    Logger::GetInstance().Info(TAG, "  Handler Memory: %d bytes", static_cast<int>(handler_memory));
    Logger::GetInstance().Info(TAG, "  Max Possible Devices: %d", MAX_IMU_DEVICES);
    
    // Hardware Configuration
    Logger::GetInstance().Info(TAG, "Hardware Configuration:");
    Logger::GetInstance().Info(TAG, "  Communication: I2C/SPI via CommChannelsManager");
    Logger::GetInstance().Info(TAG, "  Interrupt Pin: Via GpioManager");
    Logger::GetInstance().Info(TAG, "  Platform Mapping: Enabled");
    
    Logger::GetInstance().Info(TAG, "=== END IMU MANAGER STATISTICS ===");
} 
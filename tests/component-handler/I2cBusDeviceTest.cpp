/**
 * @file I2cBusDeviceTest.cpp
 * @brief Unit tests for the new I2C bus-device architecture.
 *
 * This test file verifies:
 * - I2C bus initialization and configuration
 * - Device creation and enumeration
 * - API consistency between different access methods
 * - Error handling and edge cases
 *
 * @author HardFOC Team
 * @date 2025
 */

#include <iostream>
#include <cassert>
#include <stdexcept>

// Component managers
#include "managers/CommChannelsManager.h"

// Test framework macros
#define TEST_ASSERT(condition, message) \
    do { \
        if (!(condition)) { \
            std::cerr << "TEST FAILED: " << message << std::endl; \
            return false; \
        } \
    } while (0)

#define TEST_ASSERT_EQ(expected, actual, message) \
    do { \
        if ((expected) != (actual)) { \
            std::cerr << "TEST FAILED: " << message \
                      << " (expected: " << (expected) \
                      << ", actual: " << (actual) << ")" << std::endl; \
            return false; \
        } \
    } while (0)

#define TEST_ASSERT_NOT_NULL(ptr, message) \
    do { \
        if ((ptr) == nullptr) { \
            std::cerr << "TEST FAILED: " << message << " (pointer is null)" << std::endl; \
            return false; \
        } \
    } while (0)

/**
 * @brief Test CommChannelsManager initialization.
 */
bool testCommChannelsManagerInitialization() {
    std::cout << "Testing CommChannelsManager initialization..." << std::endl;
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Test singleton pattern
    auto& comm_mgr2 = CommChannelsManager::GetInstance();
    TEST_ASSERT(&comm_mgr == &comm_mgr2, "CommChannelsManager should be singleton");
    
    // Test initialization
    bool init_result = comm_mgr.EnsureInitialized();
    TEST_ASSERT(init_result, "CommChannelsManager should initialize successfully");
    
    // Test double initialization
    bool init_result2 = comm_mgr.EnsureInitialized();
    TEST_ASSERT(init_result2, "CommChannelsManager should handle double initialization");
    
    // Test initialized state
    TEST_ASSERT(comm_mgr.IsInitialized(), "CommChannelsManager should report initialized state");
    
    std::cout << "✓ CommChannelsManager initialization tests passed" << std::endl;
    return true;
}

/**
 * @brief Test I2C bus access.
 */
bool testI2cBusAccess() {
    std::cout << "Testing I2C bus access..." << std::endl;
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Test bus reference access
    try {
        auto& i2c_bus = comm_mgr.GetI2cBus();
        std::cout << "✓ I2C bus reference obtained successfully" << std::endl;
        
        // Test device count
        std::size_t device_count = i2c_bus.GetDeviceCount();
        std::cout << "✓ I2C bus has " << device_count << " devices" << std::endl;
        
        // Compare with manager's device count
        std::size_t mgr_device_count = comm_mgr.GetI2cDeviceCount();
        TEST_ASSERT_EQ(device_count, mgr_device_count, 
                      "Bus and manager should report same device count");
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to get I2C bus: " << e.what() << std::endl;
        return false;
    }
    
    std::cout << "✓ I2C bus access tests passed" << std::endl;
    return true;
}

/**
 * @brief Test I2C device access patterns.
 */
bool testI2cDeviceAccess() {
    std::cout << "Testing I2C device access patterns..." << std::endl;
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Test device count
    std::size_t device_count = comm_mgr.GetI2cDeviceCount();
    std::cout << "✓ I2C device count: " << device_count << std::endl;
    
    // Test index-based access
    for (std::size_t i = 0; i < device_count; ++i) {
        BaseI2c* device = comm_mgr.GetI2cDevice(i);
        TEST_ASSERT_NOT_NULL(device, "I2C device should not be null");
        
        // Test ESP-specific access
        EspI2cDevice* esp_device = comm_mgr.GetEspI2cDevice(i);
        TEST_ASSERT_NOT_NULL(esp_device, "ESP I2C device should not be null");
        
        std::cout << "✓ Device " << i << " accessible via both interfaces" << std::endl;
    }
    
    // Test out-of-bounds access
    BaseI2c* invalid_device = comm_mgr.GetI2cDevice(device_count + 1);
    TEST_ASSERT(invalid_device == nullptr, "Out-of-bounds device access should return null");
    
    std::cout << "✓ I2C device access tests passed" << std::endl;
    return true;
}

/**
 * @brief Test I2C enumerated device access.
 */
bool testI2cEnumeratedAccess() {
    std::cout << "Testing I2C enumerated device access..." << std::endl;
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Test enumerated access for known devices
    BaseI2c* imu_device = comm_mgr.GetI2cDevice(I2cDeviceId::BNO08X_IMU);
    BaseI2c* gpio_device = comm_mgr.GetI2cDevice(I2cDeviceId::PCAL9555_GPIO_EXPANDER);
    
    // Note: These might be null if the devices aren't configured or available
    // but the API should handle this gracefully
    
    // Test convenience accessors
    BaseI2c* imu_convenience = comm_mgr.GetImu();
    BaseI2c* gpio_convenience = comm_mgr.GetGpioExpander();
    
    // Test ESP-specific enumerated access
    EspI2cDevice* esp_imu = comm_mgr.GetEspImu();
    EspI2cDevice* esp_gpio = comm_mgr.GetEspGpioExpander();
    
    // Test consistency between different access methods
    if (imu_device && imu_convenience) {
        TEST_ASSERT(imu_device == imu_convenience, 
                   "Enumerated and convenience IMU access should return same device");
    }
    
    if (gpio_device && gpio_convenience) {
        TEST_ASSERT(gpio_device == gpio_convenience, 
                   "Enumerated and convenience GPIO access should return same device");
    }
    
    std::cout << "✓ I2C enumerated access tests passed" << std::endl;
    return true;
}

/**
 * @brief Test legacy I2C compatibility.
 */
bool testLegacyCompatibility() {
    std::cout << "Testing legacy I2C compatibility..." << std::endl;
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Test legacy I2C access
    try {
        BaseI2c& legacy_i2c = comm_mgr.GetI2c(0);
        std::cout << "✓ Legacy I2C access works" << std::endl;
        
        // Test legacy count
        std::size_t legacy_count = comm_mgr.GetI2cCount();
        std::cout << "✓ Legacy I2C count: " << legacy_count << std::endl;
        
        // Compare with new API
        std::size_t new_count = comm_mgr.GetI2cDeviceCount();
        TEST_ASSERT_EQ(legacy_count, new_count, 
                      "Legacy and new API should report same device count");
        
    } catch (const std::exception& e) {
        std::cerr << "Legacy I2C access failed: " << e.what() << std::endl;
        return false;
    }
    
    std::cout << "✓ Legacy I2C compatibility tests passed" << std::endl;
    return true;
}

/**
 * @brief Test error handling and edge cases.
 */
bool testErrorHandling() {
    std::cout << "Testing error handling and edge cases..." << std::endl;
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Test invalid device access
    BaseI2c* invalid_device = comm_mgr.GetI2cDevice(-1);
    TEST_ASSERT(invalid_device == nullptr, "Invalid device index should return null");
    
    // Test invalid enumerated access
    BaseI2c* invalid_enum = comm_mgr.GetI2cDevice(static_cast<I2cDeviceId>(255));
    TEST_ASSERT(invalid_enum == nullptr, "Invalid device enum should return null");
    
    // Test ESP-specific invalid access
    EspI2cDevice* invalid_esp = comm_mgr.GetEspI2cDevice(-1);
    TEST_ASSERT(invalid_esp == nullptr, "Invalid ESP device index should return null");
    
    std::cout << "✓ Error handling tests passed" << std::endl;
    return true;
}

/**
 * @brief Test API consistency across access methods.
 */
bool testApiConsistency() {
    std::cout << "Testing API consistency across access methods..." << std::endl;
    
    auto& comm_mgr = CommChannelsManager::GetInstance();
    
    // Test that the same device is returned by different access methods
    std::size_t device_count = comm_mgr.GetI2cDeviceCount();
    
    for (std::size_t i = 0; i < device_count; ++i) {
        BaseI2c* device_by_index = comm_mgr.GetI2cDevice(i);
        EspI2cDevice* esp_device_by_index = comm_mgr.GetEspI2cDevice(i);
        
        if (device_by_index && esp_device_by_index) {
            // Both should point to the same underlying device
            // (esp_device_by_index should be a cast of device_by_index)
            TEST_ASSERT(static_cast<BaseI2c*>(esp_device_by_index) == device_by_index,
                       "Index-based access should return consistent devices");
        }
    }
    
    std::cout << "✓ API consistency tests passed" << std::endl;
    return true;
}

/**
 * @brief Run all tests.
 */
bool runAllTests() {
    std::cout << "=== I2C Bus-Device Architecture Tests ===" << std::endl;
    
    bool all_passed = true;
    
    all_passed &= testCommChannelsManagerInitialization();
    all_passed &= testI2cBusAccess();
    all_passed &= testI2cDeviceAccess();
    all_passed &= testI2cEnumeratedAccess();
    all_passed &= testLegacyCompatibility();
    all_passed &= testErrorHandling();
    all_passed &= testApiConsistency();
    
    if (all_passed) {
        std::cout << "=== All tests passed! ===" << std::endl;
    } else {
        std::cout << "=== Some tests failed! ===" << std::endl;
    }
    
    return all_passed;
}

/**
 * @brief Main test function.
 */
int main() {
    std::cout << "I2C Bus-Device Architecture Test Suite" << std::endl;
    std::cout << "Testing new ESP-IDF v5.5+ I2C implementation" << std::endl;
    
    try {
        bool success = runAllTests();
        return success ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Test suite failed with exception: " << e.what() << std::endl;
        return 1;
    }
}

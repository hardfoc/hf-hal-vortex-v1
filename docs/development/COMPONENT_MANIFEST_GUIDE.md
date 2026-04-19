# HardFOC HAL Component Manifest Guide

## Overview

This guide provides comprehensive documentation for the `idf_component.yml` manifest file used by the HardFOC HAL (Hardware Abstraction Layer) component. The manifest follows the [ESP-IDF Component Manager](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html) specification and enables the component to be distributed through the [ESP Component Registry](https://components.espressif.com/).

## Manifest File Structure

The `idf_component.yml` file is located in the root directory of the HardFOC HAL component and contains the following sections:

### 1. Component Metadata
```yaml
name: hf-hal-vortex-v1
version: "1.0.0"
description: |
  HardFOC Hardware Abstraction Layer for Vortex V1 platform - comprehensive 
  motor control, sensor management, and communication interface...
```

### 2. External Links
```yaml
url: "https://github.com/hardfoc/hf-hal-vortex-v1"
repository: "https://github.com/hardfoc/hf-hal-vortex-v1.git"
documentation: "https://hardfoc.dev/docs/hal"
issues: "https://github.com/hardfoc/hf-hal-vortex-v1/issues"
discussion: "https://github.com/hardfoc/hf-hal-vortex-v1/discussions"
```

### 3. Licensing and Maintenance
```yaml
license: "GPL-3.0-or-later"
maintainers:
  - "Nebiyu Tadesse <nebysma@gmail.com>"
```

### 4. Component Classification
```yaml
tags:
  - "hardfoc"           # HardFOC ecosystem
  - "motor-control"     # Motor control functionality
  - "hal"               # Hardware abstraction layer
  # ... additional tags
```

### 5. Supported Targets
```yaml
targets:
  - esp32c6             # Primary target - HardFOC Vortex V1 platform
```

### 6. Dependencies
```yaml
dependencies:
  idf: ">=5.0.0"        # ESP-IDF framework requirement
  driver: "*"           # ESP32 hardware drivers
  freertos: "*"         # Real-time operating system
  # ... additional dependencies
```

### 7. Packaging Configuration
```yaml
files:
  include:
    - "API/**/*"
    - "component-handlers/**/*"
    - "utils-and-drivers/**/*"
  exclude:
    - "**/docs/**"
    - "**/examples/**"
    - "**/tests/**"
    # ... additional exclusions
```

## Key Features of the Manifest

### Comprehensive Dependency Management

The manifest includes all necessary ESP-IDF dependencies organized by category:

#### Core Framework Dependencies
- **ESP-IDF Framework**: `idf: ">=5.0.0"` - Minimum ESP-IDF version requirement
- **Core Components**: `driver`, `freertos`, `nvs_flash`, `esp_timer`, etc.
- **Hardware Support**: `hal`, `soc`, `esp_hw_support`

#### Communication Dependencies
- **Peripheral Drivers**: `esp_adc`, `esp_gpio`, `esp_i2c`, `esp_spi`, `esp_can`, `esp_uart`
- **Power Management**: `esp_pm`, `esp_pwm`, `esp_pio`

#### Network and Connectivity
- **WiFi and Network**: `esp_wifi`, `esp_netif`, `esp_event`
- **OTA and HTTP**: `esp_ota`, `esp_http_client`, `esp_https_ota`, `esp_websocket_client`, `esp_http_server`

### Intelligent File Packaging

The manifest uses a sophisticated include/exclude system to optimize package size:

#### Include Patterns
- **Source Code**: All necessary source files and headers
- **Configuration**: `CMakeLists.txt`, `idf_component.yml`
- **Documentation**: `LICENSE`, `README.md`, `DOCUMENTATION_INDEX.md`

#### Exclude Patterns
- **Development Files**: `.github/`, `.git/`, `.idea/`, `.vscode/`
- **Documentation**: `docs/`, `examples/`, `tests/`
- **Build Artifacts**: `build/`, `dist/`, `managed_components/`
- **Cache Files**: `__pycache__/`, `*.pyc`, `.DS_Store`

### Rich Tagging System

The component uses 17 comprehensive tags for discoverability:

```yaml
tags:
  - "hardfoc"           # HardFOC ecosystem
  - "motor-control"     # Motor control functionality
  - "hal"               # Hardware abstraction layer
  - "driver"            # Device drivers
  - "esp-idf"           # ESP-IDF framework
  - "foc"               # Field-oriented control
  - "bldc"              # Brushless DC motors
  - "sensor"            # Sensor management
  - "gpio"              # GPIO control
  - "spi"               # SPI communication
  - "i2c"               # I2C communication
  - "uart"              # UART communication
  - "can"               # CAN communication
  - "adc"               # ADC functionality
  - "rtos"              # RTOS integration
  - "vortex-v1"         # Vortex V1 platform
```

## Usage in ESP-IDF Projects

### Adding the Component as a Dependency

Users can add the HardFOC HAL to their projects using the ESP-IDF Component Manager:

#### Via Command Line
```bash
# Add to main component
idf.py add-dependency hardfoc/hf-hal-vortex-v1^1.0.0

# Add to specific component
idf.py add-dependency --component=my_component hardfoc/hf-hal-vortex-v1^1.0.0
```

#### Via Manifest File
```yaml
# In project's idf_component.yml
dependencies:
  hardfoc/hf-hal-vortex-v1: "^1.0.0"
```

### Using the Component

Once added, users can include and use the HardFOC HAL:

```cpp
#include "API/Vortex.h"

// Get the unified API instance
auto& vortex = Vortex::GetInstance();

// Initialize all systems
if (vortex.EnsureInitialized()) {
    // Access motor controllers
    auto& motors = vortex.motors;
    
    // Access IMU sensors
    auto& imu = vortex.imu;
    
    // Access GPIO management
    auto& gpio = vortex.gpio;
    
    // Use any component through unified interface
}
```

## Version Management

### Versioning Strategy

The component follows [semantic versioning](https://semver.org/) principles:

- **Major Version (x.0.0)**: Breaking changes, new APIs
- **Minor Version (x.y.0)**: New features, backward compatible
- **Patch Version (x.y.z)**: Bug fixes and minor improvements

### Current Version: 1.0.0

This version represents the first stable release of the HardFOC HAL with:
- Complete API layer implementation
- All 8 component handlers
- Full ESP32 support
- Comprehensive documentation

### Version Compatibility

- **ESP-IDF**: Requires version 5.0.0 or later
- **Target**: ESP32 (primary), extensible to other ESP32 variants
- **C++ Standard**: C++17 or later

## Maintenance Guidelines

### Updating Dependencies

When updating ESP-IDF dependencies:

1. **Test Compatibility**: Ensure all features work with new ESP-IDF versions
2. **Update Version Requirements**: Modify `idf: ">=5.0.0"` if needed
3. **Add New Dependencies**: Include any new ESP-IDF components required
4. **Remove Obsolete Dependencies**: Clean up unused dependencies

### Adding New Features

When adding new features to the HAL:

1. **Update Tags**: Add relevant tags for new functionality
2. **Update Description**: Modify the description to reflect new capabilities
3. **Update Dependencies**: Add any new ESP-IDF dependencies required
4. **Update Architecture Documentation**: Modify the component architecture overview

### Package Optimization

Regularly review and optimize the package:

1. **Monitor Package Size**: Keep the distribution package as small as possible
2. **Review Include/Exclude Patterns**: Ensure all necessary files are included
3. **Update Exclusions**: Add new file types to exclude patterns as needed
4. **Test Package Integrity**: Verify all required files are present in the package

## Publishing to ESP Component Registry

### Prerequisites

1. **ESP-IDF Account**: Register at [components.espressif.com](https://components.espressif.com/)
2. **Component Validation**: Ensure the component builds and works correctly
3. **Documentation**: Complete all documentation and examples
4. **License Compliance**: Verify all licenses are properly specified

### Publishing Process

1. **Create Account**: Register for ESP Component Registry access
2. **Upload Component**: Use the ESP Component Manager tools to upload
3. **Review Process**: Component will be reviewed by ESP-IDF team
4. **Publication**: Once approved, component becomes available in registry

### Post-Publication

1. **Monitor Usage**: Track component downloads and usage
2. **Respond to Issues**: Address user issues and feedback
3. **Regular Updates**: Maintain and update the component
4. **Version Management**: Follow semantic versioning for releases

## Troubleshooting

### Common Issues

#### Dependency Resolution Errors
```bash
# Error: Component not found
# Solution: Verify component name and version in registry
idf.py add-dependency hardfoc/hf-hal-vortex-v1^1.0.0
```

#### Build Errors
```bash
# Error: Missing dependencies
# Solution: Run reconfigure to update dependencies
idf.py reconfigure
```

#### Package Size Issues
- **Problem**: Package too large
- **Solution**: Review exclude patterns in manifest
- **Action**: Add more file types to exclude list

### Debugging Manifest Issues

1. **Validate YAML Syntax**: Use YAML validator to check syntax
2. **Test Locally**: Test component locally before publishing
3. **Check Dependencies**: Verify all dependencies are available
4. **Review Logs**: Check ESP Component Manager logs for errors

## Best Practices

### Manifest File Best Practices

1. **Clear Documentation**: Provide comprehensive descriptions
2. **Accurate Tags**: Use relevant and specific tags
3. **Minimal Dependencies**: Only include necessary dependencies
4. **Optimized Packaging**: Exclude unnecessary files
5. **Version Management**: Follow semantic versioning strictly

### Component Development Best Practices

1. **Regular Updates**: Keep dependencies up to date
2. **Testing**: Test with multiple ESP-IDF versions
3. **Documentation**: Maintain comprehensive documentation
4. **Examples**: Provide clear usage examples
5. **Backward Compatibility**: Maintain compatibility when possible

## References

- [ESP-IDF Component Manager Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html)
- [ESP Component Registry](https://components.espressif.com/)
- [Manifest File Format Reference](https://docs.espressif.com/projects/idf-component-manager/en/latest/reference/manifest_file.html)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)

## Support

For issues and questions related to the HardFOC HAL component manifest:

- **GitHub Issues**: [https://github.com/hardfoc/hf-hal-vortex-v1/issues](https://github.com/hardfoc/hf-hal-vortex-v1/issues)
- **Discussions**: [https://github.com/hardfoc/hf-hal-vortex-v1/discussions](https://github.com/hardfoc/hf-hal-vortex-v1/discussions)
- **Documentation**: [https://hardfoc.dev/docs/hal](https://hardfoc.dev/docs/hal)

---

*This guide is maintained as part of the HardFOC HAL component and should be updated when the manifest structure changes.*

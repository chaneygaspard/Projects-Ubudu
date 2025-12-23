# Error Estimation Repository

A comprehensive repository containing both **UWB (Ultra-Wideband)** and **BLE (Bluetooth Low Energy)** error estimation implementations for indoor positioning systems.

## 🏗️ Repository Structure

```
error_estimation/
├── uwb_error_estimation/          # UWB Error Estimation Project
│   ├── cppversion/                # C++ Implementation
│   │   ├── main.cpp              # MQTT-based UWB processing
│   │   ├── models.cpp/h          # Core UWB models (Anchor, TagSystem)
│   │   ├── utils.cpp/h           # Utility functions
│   │   ├── tests/                # Comprehensive test suite
│   │   │   ├── test_mqtt_runner.cpp  # MQTT stream processing test
│   │   │   ├── generate_test_data.py # Test data generation
│   │   │   └── plot_results.py       # Visualization generator
│   │   └── Makefile              # Build system
│   └── pyversion/                # Python Implementation
│       ├── main.py               # Main processing script
│       ├── models.py             # UWB models and algorithms
│       ├── tests/                # Visualization test suite
│       └── requirements.txt      # Python dependencies
├── ble_error_estimation/          # BLE Error Estimation Project
│   ├── CppVersion/               # C++ Implementation
│   │   ├── main.cpp              # BLE RSSI processing
│   │   ├── models.cpp/h          # BLE models and Kalman filtering
│   │   ├── metrics.cpp/h         # Error metrics and analysis
│   │   └── tests/                # Performance and unit tests
│   └── PyVersion/                # Python Implementation
│       ├── models.py             # BLE models
│       ├── kalman.py             # Kalman filter implementation
│       └── tests/                # Python test suite
└── docs/                         # Documentation (if needed)
```

## 🎯 Projects Overview

### UWB Error Estimation
- **Technology**: Ultra-Wideband positioning
- **Use Case**: High-precision indoor positioning
- **Features**: 
  - MQTT stream processing
  - Kalman filtering
  - Anchor health monitoring
  - GDOP (Geometric Dilution of Precision) analysis
  - CEP95 error estimation
- **Languages**: C++ (production), Python (testing/visualization)

### BLE Error Estimation  
- **Technology**: Bluetooth Low Energy RSSI
- **Use Case**: Proximity-based positioning
- **Features**:
  - RSSI-based distance estimation
  - Kalman filtering for position smoothing
  - Performance metrics and analysis
  - MQTT integration
- **Languages**: C++ (production), Python (research)

## 🚀 Quick Start

### Prerequisites

#### For C++ Development:
```bash
# Ubuntu/WSL
sudo apt update
sudo apt install -y build-essential pkg-config cmake
sudo apt install -y libeigen3-dev nlohmann-json3-dev
sudo apt install -y libcurl4-openssl-dev libmosquitto-dev
```

#### For Python Development:
```bash
pip3 install -r uwb_error_estimation/pyversion/requirements.txt
pip3 install matplotlib pandas seaborn numpy
```

### Running UWB Error Estimation

#### C++ Version:
```bash
cd uwb_error_estimation/cppversion

# Install dependencies (WSL/Ubuntu)
./install_dependencies.sh

# Build and run MQTT test
make mqtt-test

# Or build main application
make all
./uwb_error_estimation
```

#### Python Version:
```bash
cd uwb_error_estimation/pyversion

# Run main application
python3 main.py

# Run visualization tests
python3 run_tests.py
```

### Running BLE Error Estimation

#### C++ Version:
```bash
cd ble_error_estimation/CppVersion

# Build and run
make all
./ble_error_estimation

# Run tests
make test
```

#### Python Version:
```bash
cd ble_error_estimation/PyVersion

# Run main application
python3 mqtt_runner.py

# Run tests
python3 -m pytest tests/
```

## 📊 Features

### UWB Error Estimation
- **Real-time Processing**: MQTT stream processing with microsecond timing
- **Advanced Filtering**: Kalman filter with dynamic covariance
- **Error Analysis**: Comprehensive statistical analysis with CEP95, GDOP
- **Health Monitoring**: EWMA-based anchor health tracking
- **Visualization**: Performance plots and error distribution analysis

### BLE Error Estimation
- **RSSI Processing**: Distance estimation from signal strength
- **Kalman Filtering**: Position smoothing and prediction
- **Performance Metrics**: Comprehensive error analysis
- **Visualization**: BLE RSSI plotting and analysis tools

## 🧪 Testing

### UWB Testing
```bash
# Run comprehensive MQTT stream test
cd uwb_error_estimation/cppversion
make mqtt-test

# Run Python visualization tests
cd uwb_error_estimation/pyversion
python3 tests/run_visualization_tests.py
```

### BLE Testing
```bash
# Run C++ performance tests
cd ble_error_estimation/CppVersion
make test

# Run Python tests
cd ble_error_estimation/PyVersion
python3 -m pytest tests/
```

## 📈 Performance

### UWB Performance Metrics
- **Processing Speed**: 1-5ms per message
- **Throughput**: 200-1000 messages/second
- **Accuracy**: CEP95 < 1m in optimal conditions
- **Memory Usage**: <100MB for full processing

### BLE Performance Metrics
- **RSSI Processing**: <1ms per measurement
- **Distance Accuracy**: ±2-5m typical
- **Update Rate**: 1-10Hz depending on configuration

## 📚 Documentation

- [`uwb_error_estimation/README.md`](uwb_error_estimation/README.md) - UWB project details
- [`uwb_error_estimation/cppversion/tests/MQTT_TEST_README.md`](uwb_error_estimation/cppversion/tests/MQTT_TEST_README.md) - MQTT test documentation
- [`ble_error_estimation/CppVersion/README.md`](ble_error_estimation/CppVersion/README.md) - BLE C++ documentation
- [`ble_error_estimation/PyVersion/README.md`](ble_error_estimation/PyVersion/README.md) - BLE Python documentation

## 🛠️ Development

### Adding New Features
1. Choose the appropriate project (UWB or BLE)
2. Implement in both C++ and Python versions if applicable
3. Add comprehensive tests
4. Update documentation
5. Submit pull request

### Code Style
- **C++**: Follow Google C++ Style Guide
- **Python**: Follow PEP 8
- **Comments**: Comprehensive documentation for all public interfaces

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📋 Requirements

### System Requirements
- **OS**: Linux (Ubuntu 20.04+), WSL2, macOS
- **Compiler**: GCC 9+ or Clang 10+
- **Python**: 3.8+
- **Memory**: 4GB+ RAM recommended
- **Storage**: 1GB+ free space

### Dependencies
- **Eigen3**: Linear algebra library
- **nlohmann-json**: JSON parsing
- **libcurl**: HTTP client
- **libmosquitto**: MQTT client
- **matplotlib, pandas, seaborn**: Python visualization

## 📄 License

[License information redacted]

## 👥 Team

- **Developer**: [Name redacted]
- **Project**: Indoor Location Services (ILS)

## 🔗 Related Projects

- **ILS Worker**: Computation engine for ILS position calculations
- **UWB Floor Detection**: XGBoost-based floor detection system

---

For detailed documentation on each component, please refer to the respective README files in each project directory.


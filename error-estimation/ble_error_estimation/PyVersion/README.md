# BLE RSSI Positioning System

A real-time Bluetooth Low Energy (BLE) positioning system that processes RSSI (Received Signal Strength Indicator) measurements from Ubudu positioning infrastructure to estimate position error bounds with adaptive anchor health monitoring and Kalman filter-based parameter estimation.

## Features

- **Real-time MQTT processing**: Live processing of Ubudu positioning data streams
- **Dynamic anchor discovery**: Automatic detection and initialization of anchors from MQTT messages
- **Adaptive path-loss modeling**: Kalman filter-based parameter estimation for RSSI₀ and path-loss exponent (n)
- **Anchor health monitoring**: EWMA-based detection of faulty anchors with automatic filtering
- **Error radius calculation**: 95% confidence error bounds (CEP95) based on statistical analysis
- **API integration**: Automatic fetching of anchor positions from Ubudu configuration API
- **Comprehensive visualization**: Static plot generation for system analysis and debugging
- **Real-time data collection**: Configurable data collection periods for analysis

## System Architecture

### Core Components

- **`mqtt_runner.py`**: Main MQTT client for real-time processing of Ubudu positioning data
- **`models.py`**: Data models for Anchors, Tags, and Path-loss models with Kalman integration
- **`metrics.py`**: TagSystem class and anchor health management with EWMA filtering
- **`kalman.py`**: Kalman filter implementation for adaptive RSSI₀ and n parameter estimation
- **`utils.py`**: Utility functions for distance calculations and statistical operations
- **`plot_ble_rssi.py`**: Data collection and visualization tool for system analysis

### Key Algorithms

1. **Path-loss Model**: `RSSI = RSSI₀ - 10n·log₁₀(d/d₀) + noise`
2. **Health Monitoring**: EWMA filter with λ=0.05 to track anchor reliability  
3. **Confidence Scoring**: Student's t-distribution for statistical assessment
4. **Anchor Filtering**: Automatic exclusion of unreliable anchors (EWMA ≥ 8.0)
5. **Parameter Adaptation**: Kalman filter updates for RSSI₀ and path-loss exponent

## Installation

### Prerequisites

- Python 3.8+
- Access to Ubudu MQTT broker (`ils-paris.ubudu.com`)
- Ubudu API credentials

### Dependencies

```bash
pip install paho-mqtt numpy matplotlib scipy requests
```

### Testing

```bash
pip install pytest
python -m pytest tests/
```

## Usage

### Real-time MQTT Processing

```bash
python mqtt_runner.py
```

The system will:
1. Connect to Ubudu MQTT broker at `ils-paris.ubudu.com`
2. Subscribe to `engine/6ba4a2a3-0/positions` for positioning data
3. Dynamically discover and initialize anchors from first message
4. Process tag positioning data with adaptive parameter estimation
5. Publish error estimates to `engine/6ba4a2a3-0/error_estimates`

### Data Collection and Visualization

```bash
python plot_ble_rssi.py
```

Collects data for a specified period (default 30 seconds) and generates comprehensive plots:
1. **Error Radius Over Time**: CEP95 confidence bounds for each tag
2. **Tag Movement Trajectories**: 2D movement paths with start/end markers
3. **Final System State**: Current positions with error circles and anchor layout
4. **Error Statistics Summary**: Min/max/average/final error comparison
5. **Anchor n_var Over Time**: Path-loss exponent evolution
6. **Current Anchor States**: n_var vs EWMA scatter plot

## Configuration

### MQTT Settings

Edit `mqtt_runner.py`:
```python
BROKER = "ils-paris.ubudu.com"
PORT = 1883
TAG_POSITION_STREAM = "engine/6ba4a2a3-0/positions"
TOPIC_OUT = "engine/6ba4a2a3-0/error_estimates"
CLIENT_ID = "ble_rssi_probability_model"
```

### API Configuration

```python
ANCHOR_INIT_BASE = "https://ils-paris.ubudu.com/confv1/api/dongles?macAddress={}"
anch_api_auth = ("admin", "ubudu_rocks")
```

### System Parameters

Key parameters in `metrics.py`:
- `EWMA_THRESHOLD = 8.0`: Health threshold for anchor filtering
- `LAMBDA = 0.05`: EWMA learning rate in `update_health()`
- `deltaR = 7.0`: RSSI difference threshold for anchor gating
- `T_vis = 6000`: Visibility timeout for anchors (seconds)

### Visualization Settings

Edit `plot_ble_rssi.py`:
```python
COLLECTION_TIME = 30  # Data collection period in seconds
BROKER = "ils-paris.ubudu.com"
POSITION_TOPIC = "engine/6ba4a2a3-0/positions"
ERROR_TOPIC = "engine/6ba4a2a3-0/error_estimates"
```

## Message Formats

### Input Message Format

Real Ubudu positioning data from `engine/6ba4a2a3-0/positions`:

```json
{
    "timestamp": 1751374881169,
    "tag": {
        "mac": "c00fbe457cd3",
        "id": "31955"
    },
    "location": {
        "position": {
            "x": 5.92,
            "y": 2.21,
            "z": 0.0,
            "quality": "normal",
            "used_anchors": [
                {"mac": "ce59ac2d9cc5", "id": "404", "rssi": -57.0, "cart_d": 1.0},
                {"mac": "e7a7f022204d", "id": "blank", "rssi": -59.47, "cart_d": 2.07}
            ],
            "unused_anchors": [
                {"mac": "d39d76bbc21b", "id": "ea9", "rssi": -66.19, "cart_d": 4.67}
            ]
        }
    }
}
```

### Output Message Format

Error estimates published to `engine/6ba4a2a3-0/error_estimates`:

```json
{
    "tag_mac": "c00fbe457cd3",
    "error_estimate": 1.25,
    "anchors": [
        {
            "mac": "ce59ac2d9cc5",
            "n_var": 2.1,
            "ewma": 3.2
        }
    ],
    "warning_anchors": [],
    "faulty_anchors": ["d39d76bbc21b"]
}
```

## Dynamic Anchor Discovery

The system automatically discovers anchors from the first MQTT message:

1. **First Message Processing**: Extracts MAC addresses from `used_anchors` and `unused_anchors`
2. **API Fetching**: Queries Ubudu API for each discovered anchor's position
3. **Initialization**: Creates `Anchor` objects with coordinates and default parameters
4. **Adaptive Learning**: Each anchor's RSSI₀ and n parameters evolve via Kalman filtering

## Anchor Health Monitoring

### EWMA Filter Implementation

Each anchor maintains health via exponentially weighted moving average:
```python
ewma = λ·z² + (1-λ)·ewma_prev
```

### Health Classification

- **Healthy**: `ewma < 4.0` (used in calculations)
- **Warning**: `4.0 ≤ ewma < 8.0` (flagged but still used)
- **Faulty**: `ewma ≥ 8.0` (excluded from calculations)

### Filtering Logic

Anchors are excluded if:
1. Health threshold exceeded (`ewma ≥ 8.0`)
2. Signal too weak (`RSSI_difference > deltaR`)
3. Stale data (`time_since_last_seen > T_vis`)

## Testing

### Test Structure

- `tests/test_models.py`: Anchor, Tag, and PathLossModel tests (373 lines)
- `tests/test_metrics.py`: TagSystem and health monitoring tests (615 lines)
- `tests/test_kalman.py`: Kalman filter functionality tests (228 lines)
- `tests/test_utils.py`: Utility function tests (277 lines)

### Running Tests

```bash
# Run all tests
python -m pytest tests/

# Run specific test file
python -m pytest tests/test_models.py

# Verbose output
python -m pytest tests/ -v
```

## Performance Characteristics

- **Anchor Discovery**: Automatic on first message
- **Learning Time**: 10-15 messages for parameter convergence
- **Health Detection**: Responds to anchor failures within 5-10 messages
- **Real-time Processing**: Sub-second message processing latency
- **Error Convergence**: CEP95 bounds stabilize as bad anchors are filtered

## Troubleshooting

### Common Issues

1. **"No anchor found for MAC address"**: API connectivity or invalid MAC
2. **High error estimates**: Check anchor health status and EWMA values
3. **MQTT connection failures**: Verify broker address and credentials
4. **Empty plots**: Ensure MQTT data is flowing during collection period

### Debug Features

- Anchor discovery logging with MAC address extraction
- EWMA, RSSI₀, and n parameter evolution tracking
- Message count tracking per anchor
- Warning/faulty anchor identification in output

### System Monitoring

Monitor anchor ages (message counts) for processing verification:
```
DEBUG - Used anchors in message: ['ce59ac2d9cc5', 'e7a7f022204d']
DEBUG - Unused anchors in message: ['d39d76bbc21b']
Initialized 4 anchors
```

## API Integration

### Anchor Position Fetching

Automatic position retrieval from Ubudu configuration API:
```python
# API endpoint
https://ils-paris.ubudu.com/confv1/api/dongles?macAddress={mac}

# Authentication
("admin", "ubudu_rocks")

# Response format
[{"x": 0.0, "y": 0.0, "z": 2.5, ...}]
```

## Package Structure

### Exports

The package exports key classes via `__init__.py`:
```python
from ble_rssi import Anchor, Tag, PathLossModel, TagSystem
```

### Version

Current version: `0.1.0`

## License

[Your License Here]

## Contributing

[Your contribution guidelines here]

# Fregate RTLS Testing Suite

This repository contains Python scripts for testing and analyzing Real-Time Location System (RTLS) positioning accuracy in a multi-floor environment.

## Overview

The scripts in this repository are designed to work with an RTLS system that tracks positioning tags across two floors (downstairs and mezzanine) using MQTT communication and UWB (Ultra-Wideband) technology.

## Files

### get_tag_macs.py

A utility script that connects to the MQTT broker and collects MAC addresses from positioning tags. This script listens to position data messages and extracts the MAC addresses associated with known tag IDs, which can then be used for configuration in other testing scripts.

### floor_success_rate.py

The main testing script that evaluates the accuracy of the RTLS positioning system across two floors. This comprehensive tool performs the following functions:

**Core Functionality:**
- **API Integration**: Makes periodic POST requests to the BLE communications REST API to trigger UWB advertising from positioning tags
- **MQTT Monitoring**: Connects to an MQTT broker to receive real-time position data from the RTLS engine
- **Floor-Based Validation**: Validates that tags are being detected on their correct floors by comparing actual map IDs with expected map IDs
- **Success Rate Calculation**: Tracks and calculates positioning accuracy rates both overall and per floor

**Key Features:**
- **Dual-Floor Support**: Separately tracks success rates for downstairs (map ID: 682c66de8cde618ce1270230) and mezzanine (map ID: 682c66f08cde618ce127025e) floors
- **Tag Management**: Monitors 60 positioning tags (30 per floor) with configurable message limits per tag
- **Automated API Requests**: Periodically triggers tag broadcasting to ensure continuous data flow (every 100 seconds with 90-second timeout)
- **Real-Time Statistics**: Displays live success/failure counts and percentages during operation
- **Comprehensive Reporting**: Provides detailed final statistics including runtime, per-floor accuracy, message distribution, and non-responsive tags

**Configuration:**
- **MQTT Connection**: Connects to ils-paris.ubudu.com:1883 using the topic pattern "engine/+/positions"
- **Tag Limits**: Processes up to 3 messages per tag to prevent skewing results
- **API Endpoint**: Uses the BLE communications REST API for tag activation
- **Error Handling**: Robust error handling for network issues, JSON parsing, and unexpected data

**Output Statistics:**
- Overall positioning success rate percentage
- Floor-specific success rates (downstairs vs mezzanine)
- Runtime duration and total messages processed
- Tag response statistics (average, maximum, minimum message counts)
- List of non-responsive tags for troubleshooting

This script is essential for validating RTLS deployment quality and identifying positioning accuracy issues in multi-floor environments.

## Usage

1. Ensure you have the required dependencies installed:
   ```bash
   pip install paho-mqtt requests
   ```

2. Run the floor success rate test:
   ```bash
   python floor_success_rate.py
   ```

3. To collect tag MAC addresses:
   ```bash
   python get_tag_macs.py
   ```

## Requirements

- Python 3.6+
- paho-mqtt
- requests
- Access to the RTLS MQTT broker and REST API

## Configuration

The scripts are pre-configured with the appropriate broker settings and tag configurations for the current deployment. Modify the broker host, port, and tag lists as needed for different environments.

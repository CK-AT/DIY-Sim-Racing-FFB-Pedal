# Gateway Auto-Detection

## Overview

The SimHub plugin now includes automatic gateway detection, which scans all available COM ports to identify connected DIY FFB gateways. This eliminates the need to manually search for the correct COM port.

## Features

### 1. Manual Gateway Scan

Click the **"Scan for Gateways"** button in the System tab to manually scan all COM ports.

**How it works:**
- Scans all available COM ports sequentially
- Shows progress in the debug output (e.g., "Scanning COM3 (2/8)...")
- Identifies devices that respond as gateways
- Displays detailed information about discovered gateways:
  - COM port name
  - Gateway ID (1-4)
  - Firmware version
  - Board type
  - Device UID

**What happens after scan:**
- If gateways are found, you'll see a dialog with details about the first gateway
- You can choose to connect immediately or cancel
- The COM port dropdown is populated with discovered gateway ports only

### 2. Auto-Scan on Startup

Enable **"Auto Scan on Startup"** checkbox to automatically search for gateways when SimHub starts.

**Benefits:**
- No manual intervention needed
- Works seamlessly with "Auto Reconnect" feature
- Automatically updates the selected COM port if a gateway is found

**How it works:**
- Runs in the background when the plugin UI is initialized
- If a gateway is found, it updates the ESPNow_port setting
- If "Auto Reconnect" is also enabled, it will automatically connect to the discovered gateway

### 3. Auto Reconnect

The existing **"Auto Reconnect"** feature now works even better with auto-detection.

**Usage scenarios:**

| Auto Scan | Auto Reconnect | Behavior |
|-----------|----------------|----------|
| ☐ | ☐ | Manual port selection and connection |
| ☑ | ☐ | Finds gateway on startup, but doesn't connect |
| ☐ | ☑ | Connects to saved port if available |
| ☑ | ☑ | Finds gateway and connects automatically (fully automatic!) |

## Technical Details

### Gateway Detection Process

The scanner performs the following steps for each COM port:

1. **Open the port** at 3,000,000 baud (standard ESP32 speed)
2. **Send DeviceInfoRequest** messages for gateway IDs 1-4
3. **Wait for DeviceInfo response** (250ms timeout per gateway ID)
4. **Check if response indicates gateway** (vs. axis device)
5. **Extract device information** (firmware, board, UID)
6. **Close the port** and move to next

**Timeout values:**
- Per gateway ID: 250ms
- Total per port: ~1 second (4 gateway IDs × 250ms)
- Typical full scan (8 ports): ~8 seconds

### Why It's Safe

- **Non-destructive**: Only sends read-only DeviceInfoRequest messages
- **Timeout protection**: Doesn't hang on unresponsive ports
- **Exception handling**: Gracefully handles ports that are in use
- **Cancellable**: Scan can be cancelled mid-operation

### Error Handling

**Port in use:**
- Silently skipped (logged to SimHub debug output)
- Common when other software has the port open

**Timeout:**
- Indicates port has a device, but not a gateway
- Normal behavior for non-gateway serial devices

**Permission denied:**
- Port requires elevated privileges
- Rare on Windows unless using system ports

## Usage Examples

### Example 1: First-Time Setup

1. Install the plugin with no previous gateway configuration
2. Enable "Auto Scan on Startup" checkbox
3. Enable "Auto Reconnect" checkbox
4. Restart SimHub
5. Gateway is automatically detected and connected

### Example 2: Multiple Gateways

If you have multiple gateways (e.g., different rigs):

1. Click "Scan for Gateways"
2. The scan finds all connected gateways
3. The dropdown shows all gateway ports (e.g., COM3, COM5)
4. Select the desired gateway from the dropdown
5. Click "Connect"

**Note:** Currently, the auto-connect feature connects to the first gateway found. If you have multiple gateways, disable "Auto Scan" and manually select your preferred port.

### Example 3: Development/Debugging

Scan results are logged to SimHub's log file:

```
[Gateway] Gateway 1 on COM3 (FW: v2.8.1, Board: PCB_V13)
[GatewayScanner] Auto-detected gateway on COM3 (ID: _1)
```

Check `C:\Program Files (x86)\SimHub\Logs\SimHub.txt` for detailed scan information.

## Troubleshooting

### "No gateways found"

**Possible causes:**
1. Gateway not powered on
2. USB cable not connected
3. Gateway firmware doesn't respond to DeviceInfoRequest (older firmware)
4. Another application has the port open

**Solutions:**
- Verify gateway is powered and connected
- Check Windows Device Manager for COM port presence
- Close other serial terminal applications
- Try "Update Serial Port List" first to see available ports

### Scan takes too long

Normal scan time: 1-2 seconds per COM port.

If longer:
- Many virtual COM ports installed (e.g., from Bluetooth devices)
- Some ports are slow to respond

**Solution:** The scan is cancellable - click "Scan for Gateways" again to cancel.

### Wrong gateway selected (multiple gateways)

The auto-scan always selects the **first** gateway found (lowest COM port number).

**Solution:**
1. Disable "Auto Scan on Startup"
2. Manually run scan to see all gateways
3. Select your preferred gateway from the dropdown
4. Enable "Auto Reconnect" only

### Gateway disconnects and reconnects

This is separate from auto-detection - the gateway lost connection.

**Check:**
- USB cable quality
- Power supply stability
- SimHub log for disconnect/reconnect messages

## Future Enhancements

Possible improvements (not yet implemented):

- **Gateway preference list**: Choose which gateway to connect to when multiple are found
- **Remember gateway by UID**: Connect to specific device regardless of COM port
- **Background re-scan**: Periodically search for new gateways
- **Gateway list UI**: Show all discovered gateways in a table

## Code Architecture

For developers:

- **GatewayScanner.cs**: Core scanning logic
- **DiyFfbPluginUI.xaml.cs**: UI integration (btn_scan_gateways_Click)
- **DiyFfbPluginSettings.cs**: Settings storage (Pedal_ESPNow_auto_scan_flag)

Key classes:
- `GatewayScanner`: Async scanner with progress events
- `DiscoveredGateway`: Data class for gateway info
- `ProtobufSerial<Message>`: Serial communication wrapper

## See Also

- [Plugin Design](Plugin_Design.md) - Overall plugin architecture
- ESP32 firmware: `CommManager.cpp` - DeviceInfo message handling

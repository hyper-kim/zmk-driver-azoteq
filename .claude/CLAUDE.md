# ZMK Azoteq IQS5xx Trackpad Driver

## Project Overview
This is a ZMK (Zephyr-based Mechanical Keyboard) module that provides driver support for the Azoteq IQS5xx capacitive trackpad controller. The driver enables trackpad functionality including:
- Single finger mouse movement and clicking
- Two finger scrolling and gestures
- Three finger swipe gestures
- Tap-and-hold for drag operations

## Key Files
- `src/iqs5xx.c` - Main driver for IQS5xx chip (I2C communication, register configuration)
- `src/trackpad.c` - Top-level trackpad event processing and gesture routing
- `src/single_finger.c` - Single finger gesture handling (mouse movement, clicks, drag)
- `src/two_finger.c` - Two finger gesture handling (scrolling, zoom)
- `src/three_finger.c` - Three finger gesture handling (swipes for navigation)
- `src/coordinate_transform.c` - Coordinate transformation and rotation
- `include/iqs5xx.h` - Driver interface and register definitions
- `include/gesture_handlers.h` - Gesture state machine and configuration

## Architecture
1. **Hardware Layer (iqs5xx.c)**: Handles I2C communication with the trackpad chip
2. **Event Processing (trackpad.c)**: Routes events to appropriate gesture handlers
3. **Gesture Handlers (single_finger.c, two_finger.c, three_finger.c)**: Process gestures and generate input events

## Known Issues & Platform Differences

### macOS Performance Issues
- Mouse movement can appear laggy or "jumpy" on macOS (especially M1/M2/M3 Macs with ProMotion displays)
- Root cause: macOS has different USB HID event handling and higher display refresh rates (120Hz)
- The 8ms event rate limiting in `trackpad.c` can cause events to batch up
- Solution: Use lower rate limiting (2-4ms) and adjust movement thresholds

### Windows vs macOS
- Windows handles buffered mouse events differently, providing smoother interpolation
- macOS is more sensitive to event timing and requires more frequent updates

## Configuration Parameters

### Hardware Refresh Rates (iqs5xx.c)
- `activeRefreshRate`: 5ms - How often the chip scans when active
- `idleRefreshRate`: 20ms - How often the chip scans when idle

### Software Rate Limiting (trackpad.c)
- Movement event rate limit: 3ms (~333Hz update rate for macOS 120Hz displays)
- Idle mode threshold: 16ms (60Hz when no activity)
- **Critical**: Lower rate limits cause smooth movement on macOS, higher values cause "jumping"

### Movement Processing (gesture_handlers.h)
- `MOVEMENT_THRESHOLD`: 0.3f - Minimum movement before reporting
- Mouse sensitivity: Configurable via devicetree (default 128)

## Testing & Debugging
- Event counting is built-in for debugging (see `event_count` in trackpad.c)
- I2C error recovery logic handles transient communication failures
- Power management handles idle/active transitions

## Development Guidelines
1. **Never skip rate limiting for movement events** - This can overwhelm the USB subsystem
2. **Always process gesture events immediately** - Don't rate-limit button presses
3. **Test on both Windows and macOS** - Performance characteristics differ significantly
4. **Be careful with I2C mutex timing** - Can cause system freezes if held too long

## Recent Changes
- Reduced various delays and event rate limits to improve responsiveness
- Fixed three-finger gesture key mappings
- Improved I2C initialization robustness
- Added idle mode detection and handling

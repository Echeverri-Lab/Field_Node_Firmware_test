# Field_Node_Firmware
Firmware for all field node sensors and management
Firmware Development (ESP-IDF)
Architecture: FreeRTOS, C/C++ Orchestrator
Strategy: Bottom-Up Development We build reliability by validating layers in order. We must trust the hardware interface (HAL) before we build independent loops (Tasks), and we must trust those loops before we tie them together (Orchestrator). Connectivity is added last to prevent network stack complexity from destabilizing the core data logger.

** Glossary & Guidelines**

BSP (Board Support Package): Think of this as the “Wiring Map” for our specific physical board.

The Chip (ESP32) is generic (it can do anything).

Our Board is specific (The Mic is wired to Pin 4).

The BSP tells the software: “On this specific BioMesh board, look for the Microphone on Pin 4.”

** Why? **
If we change the wiring next week, we only update the BSP. The main code (“Record Audio”) doesn’t need to change.
Library Selection (Build vs Buy):
Use a Library when: The protocol is complex (e.g., NMEA parsing, FAT filesystem) or provided by the vendor (e.g., esp32-camera).

Code it Yourself when: The sensor is simple (e.g., AHT20 needs just 2 I2C commands) to avoid bloat, or when you need tight control over timing/memory.

** 3.1 Foundation & Drivers (HAL) **

Goal: Create a trusted hardware abstraction layer. If the driver says it captured data, it must be true. Success Criteria: All bsp_* functions return ESP_OK and produce valid raw data in unit tests.

** 3.1 Foundation & Drivers (HAL) **
Goal: Create a trusted hardware abstraction layer. If the driver says it captured data, it must be true. Success Criteria: All bsp_* functions return ESP_OK and produce valid raw data in unit tests.

File Structure Example (Mental Model)
📂 main/
   └── app_main.c         (The Orchestrator)
📂 components/
   └── 📂 bsp/            (Board Support Package)
        ├── include/
        │   └── bsp_camera.h  (The Menu: "void bsp_camera_init()")
        └── bsp_camera.c      (The Kitchen: "gpio_set_level(PIN_5, 1)")

 Setup:
 
 Initialize Git Repo with ESP-IDF v5.x template.
 
 Configure sdkconfig (Enable PSRAM, Set CPU Freq, Partition Table).
 
 Driver Development:
 
 Component Evaluation (Pre-Code): Check esp-who, esp-adf, and arduino-esp32 limits before custom coding.
 
 Camera (bsp_camera):
 
 Integrate official esp32-camera component.
 
 Configure Pin Map for XIAO ESP32-S3 Sense.
 
 Implement init(): Initialize the camera driver with the specific pin map and XCLK.
 
 Implement capture_frame(): Capture a JPEG image and return the framebuffer pointer.
 
 Implement set_resolution(): Allow dynamic switching (e.g., QVGA for analysis, UXGA for storage).
 
 Add GPIO driver for 940nm IR LED control (PWM for brightness).
 
 Audio (bsp_audio):
 
 Implement init(): Configure i2s_config_t (Standard Mode, 16kHz) and call i2s_driver_install().
 
 Implement read_i2s_buffer(): Blocking call to fetch raw PCM data from DMA.
 
 Environment (bsp_env):
 
 Implement init(): Configure i2c_config_t (Master Mode, SDA/SCL pins) and install driver.
 
 Implement read_temp_humidity(): Send Trigger Command -> Wait 80ms -> Read 6 Bytes -> Convert to Float.
 
 GPS (bsp_gps):
 
 Implement init(): Configure uart_config_t (9600 baud, 8N1) and install driver on UART2.
 
 Integrate NMEA parser library (TinyGPS++ or minmea).
 
 Implement get_latest_fix(): Returns struct with {Latitude, Longitude, EpochTime} if valid.
 
 Storage (bsp_storage):
 
 Implement init(): Initialize SDMMC Host (1-bit mode) and call esp_vfs_fat_sdmmc_mount().
 
 Implement write_file(): Append data buffer to a specific file path.
 
 Implement disk_usage(): Use esp_vfs to return Total vs Free bytes.
 
 Implement find_oldest_file(): Scan directory to identify the oldest timestamped file.
 
 Implement delete_file(): Unlink specific file path (used for retention).
 
** 3.2 FreeRTOS Tasks (Subsystems) **

Goal: Isolate complexity into independent, crash-proof loops. Success Criteria: Each task can run overnight individually without crashing or leaking memory.

 Power Manager Task (sys_power):
 
 Create FreeRTOS Task and Queue for state change requests.
 
 Implement State Machine: IDLE -> CAPTURE -> SLEEP.
 
 Implement check_battery(): Read ADC, if V < 3.3V, force Deep Sleep.
 
 Config Wakeup: Ext0 (PIR), Timer (Schedule), Ext1 (Button).
 
 Vision Task (sys_vision):
 
 Create xQueueVisionRequests: Accepts CAPTURE_PHOTO commands.
 
 Implement take_photo(): Call bsp_camera_capture() -> Get Framebuffer.
 
 Implement save_to_sd(): Generate filename (timestamp.jpg) -> Write file.
 
 Add IR Logic: If is_night (via scheduling or sensor), toggle IR LED GPIO before capture.
 
 Audio Task (sys_audio):
 
 Allocate Ring Buffer in PSRAM (e.g., 5-second circular buffer).
 
 Create xQueueAudioTrigger: Accepts START_RECORDING commands.
 
 Implement record_clip(): Flush Ring Buffer + New Data -> SD/audio_timestamp.wav.
 
 Environment Task (sys_env):
 
 Create xQueueEnvRequests: Accepts SAMPLE_ENV commands.
 
 Implement sample_and_log(): Call bsp_env_read() -> Add timestamp -> Append to env_log.csv.
 
 System & Maintenance Task (sys_maint):
 
 Create struct sys_health_t: Uptime, Free Heap, Battery Voltage.
 
 Periodic Timer (every 1 min): Populate struct -> Print to Serial / Append to CSV log.
 
 Retention Loop: Periodically check bsp_storage_disk_usage(). If full, call bsp_storage_delete_oldest().
 
** 3.3 Orchestrator & Logic **

Goal: “Traffic Control” logic that manages the system state. The Orchestrator doesn’t know “how” to take a photo, just “when”. Success Criteria: System transitions correctly between Idle, Capture, and Sleep states based on sensor inputs.
 
 Event Loop (app_main):
 
 Create Main Event Queue (receives Msgs from ISRs and Tasks).
 
 Implement Logic:
 IF PIR_INTERRUPT: Check config.is_active_hours -> Send CAPTURE_PHOTO to Vision Queue.
 IF SCHEDULE_ENV: Send SAMPLE_ENV to Env Queue.
 IF SCHEDULE_AUDIO: Check config.audio_intervals -> Send START_RECORDING to Audio Queue.
 IF UPLOAD_TIMER: Send START_UPLOAD to Comms Queue (WiFi HaLow).
 IF SYNC_TIMER: Wake GPS -> Wait for Fix -> Update RTC -> Sleep GPS.
 IF BATTERY_LOW: Send ENTER_DEEP_SLEEP to Power Queue.
 
 Config System:
 Include JSON Parser (cJSON is built-in to IDF).
 Create config.json on SD Card:
{
  "device_id": "NODE_001",
  "active_start_hour": 6,
  "active_end_hour": 18,
  "pir_sensitivity": 0.8
}


 Implement load_config(): Read JSON -> Populate global settings struct at boot.
** 3.4 Connectivity (WiFi HaLow) **

Goal: Transmit data without destabilizing the core logger. Treat as “High Risk” - if this fails, SD card is the backup.Success Criteria: Data upload succeeds, or fails gracefully (timeout/retry) without hanging the main loop.

 Driver Porting (The “Iceberg”):
 
Phase 1: Regs: Validate SPI connection to Newracom chip (read/write ID registers).
Phase 2: HAL: Port low-level IO (SPI, ISRs) to ESP-IDF spi_master driver.
Phase 3: MAC: Bring up the 802.11ah MAC layer and scan for APs.
Phase 4: Netif: Bridge ETH frames into ESP-IDF’s LwIP stack (get an IP address).

 Comms Task:
 Proto-definition: Define simple binary or JSON packet structure.
 Store-and-Forward: If Upload Fails -> Mark file as “Pending” -> Retry next interval.
 Power Aware: Only attempt connection if Battery > 20%.
 Handshake: Simple “Ping/Ack” with Base Station/Gateway.
** 3.5 Data Management & Maintenance **
Goal: Ensure the system runs forever without filling the SD card or bricking. Success Criteria: Old files are auto-deleted when full, and firmware can update remotely.

 Data Retention Logic:
 
 Safe Storage: Use LittleFS/SPIFFS for logs/config (crash-safe) and FAT32 ONLY for media.
 
 Implement check_storage_space(): If < 100MB free, delete oldest files.
 
 Implement mark_uploaded(): Rename file (e.g., .wav -> .upl) or update SQLite DB after successful upload.
 
 OTA Update System:
 
 Enable native_ota_example component from IDF.
 
 Implement check_for_updates(): Query Base Station version manifest.
 
 Implement perform_update(): Download -> Write Partition -> Reboot.

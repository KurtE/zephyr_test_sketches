/*
 * I2C Scanner for Teensy 4.1
 * Scans all I2C ports every 3 seconds with detailed logging
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

struct i2c_port {
    const char *name;
    const char *pins;
    const struct device *dev;
    bool initialized;
    bool configured;
};

static struct i2c_port i2c_ports[] = {
    {"LPI2C1", "pins 18(SDA)/19(SCL)", NULL, false, false},
    {"LPI2C3", "pins 16(SDA)/17(SCL)", NULL, false, false},
    {"LPI2C4", "pins 24(SDA)/25(SCL)", NULL, false, false},
};

#define NUM_I2C_PORTS ARRAY_SIZE(i2c_ports)

static void initialize_i2c_ports(void)
{
    printk("\n=== I2C PORT INITIALIZATION ===\n");
    
    // Get device pointers
    i2c_ports[0].dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(lpi2c1));
    i2c_ports[1].dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(lpi2c3));
    i2c_ports[2].dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(lpi2c4));
    
    for (int i = 0; i < NUM_I2C_PORTS; i++) {
        printk("\nInitializing %s %s:\n", i2c_ports[i].name, i2c_ports[i].pins);
        
        // Check if device exists
        if (!i2c_ports[i].dev) {
            printk("  Device pointer: NULL - NOT AVAILABLE\n");
            continue;
        }
        
        printk("  Device pointer: %p\n", i2c_ports[i].dev);
        printk("  Device name: %s\n", i2c_ports[i].dev->name);
        
        // Check if device is ready
        if (!device_is_ready(i2c_ports[i].dev)) {
            printk("  Device ready: NO\n");
            continue;
        }
        
        printk("  Device ready: YES\n");
        i2c_ports[i].initialized = true;
        
        // Configure I2C at 100kHz (standard speed)
        uint32_t config = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
        printk("  Configuring at 100kHz (config=0x%08X)...\n", config);
        
        int ret = i2c_configure(i2c_ports[i].dev, config);
        if (ret == 0) {
            printk("  Configuration: SUCCESS\n");
            i2c_ports[i].configured = true;
        } else {
            printk("  Configuration: FAILED (error %d)\n", ret);
        }
    }
    
    printk("\n=== INITIALIZATION COMPLETE ===\n");
}

static void scan_i2c_port(struct i2c_port *port, int scan_number)
{
    printk("\n--- Scanning %s %s (Scan #%d) ---\n",
           port->name, port->pins, scan_number);
    
    if (!port->initialized) {
        printk("Port not initialized - skipping\n");
        return;
    }
    
    if (!port->configured) {
        printk("Port not configured - skipping\n");
        return;
    }
    
    int devices_found = 0;
    printk("Scanning addresses 0x08 to 0x77...\n");
    
    for (int addr = 0x08; addr <= 0x77; addr++) {
        uint8_t dummy;
        int ret = i2c_read(port->dev, &dummy, 1, addr);
        
        if (ret == 0) {
            printk("  Device found at 0x%02X\n", addr);
            devices_found++;
        }
        
        // Small delay between addresses
        k_msleep(5);
    }
    
    if (devices_found == 0) {
        printk("No devices found on %s\n", port->name);
    } else {
        printk("Total devices found on %s: %d\n", port->name, devices_found);
    }
}

static void perform_scan_cycle(int cycle_number)
{
    printk("\n\n========== SCAN CYCLE #%d ==========\n", cycle_number);
    printk("Timestamp: %lld ms\n", k_uptime_get());
    
    for (int i = 0; i < NUM_I2C_PORTS; i++) {
        scan_i2c_port(&i2c_ports[i], cycle_number);
        k_msleep(100); // Brief delay between ports
    }
    
    printk("\n========== CYCLE #%d COMPLETE ==========\n", cycle_number);
}

int main(void)
{
    printk("\n=== Fresh I2C Scanner for Teensy 4.1 ===\n");
    printk("Scans all I2C ports every 3 seconds\n");
    printk("Built: %s %s\n\n", __DATE__, __TIME__);
    
    // Initialize all I2C ports
    initialize_i2c_ports();
    
    // Summary of initialized ports
    printk("\n=== PORT SUMMARY ===\n");
    int ready_ports = 0;
    for (int i = 0; i < NUM_I2C_PORTS; i++) {
        if (i2c_ports[i].configured) {
            printk("%s: READY for scanning\n", i2c_ports[i].name);
            ready_ports++;
        } else {
            printk("%s: NOT READY\n", i2c_ports[i].name);
        }
    }
    
    if (ready_ports == 0) {
        printk("\nERROR: No I2C ports are ready!\n");
        return -1;
    }
    
    printk("\nStarting continuous scanning with %d port(s)...\n", ready_ports);
    printk("Press reset to stop\n");
    
    // Continuous scanning loop
    int scan_cycle = 1;
    while (1) {
        perform_scan_cycle(scan_cycle);
        
        printk("\nWaiting 3 seconds before next scan...\n");
        k_sleep(K_SECONDS(3));
        
        scan_cycle++;
    }
    
    return 0;
}
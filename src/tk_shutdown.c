#include <power/reboot.h>
#include "tk_shutdown.h"
// #include "app_energy.h"
// #include "app_mqtt.h"
// TODO: Renable and handle shutdown properly
void tk_shutdown(void)
{
    // APP_ENERGY_SHUTDOWN_M();
    // APP_MQTT_DISCONNECT_M();
    sys_reboot(0);
}
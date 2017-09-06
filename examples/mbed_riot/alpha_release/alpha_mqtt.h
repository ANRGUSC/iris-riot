#ifndef _MQTT_THREAD_H
#define _MQTT_THREAD_H

/**
 * @brief      This lists the states of the mqtt_control_thread in the sequence
 */
enum mqtt_states
{
    MQTT_DISCON              = 0,
    MQTT_CON_PUB_HW          = 1,
    MQTT_CON_MQTT_GO         = 2,
    MQTT_CON_MQTT_GO_WAIT    = 3,
    MQTT_CON_COMPLETE        = 4,
    MQTT_MBED_INIT_DONE      = 5    
};
/**
 * @brief      intializes the mqtt control thread as well as the emcute thread
 *
 * @param      stack      The stack
 * @param[in]  stacksize  The stacksize
 * @param[in]  priority   The priority
 * @param[in]  name       The name
 * @param      arg        The argument
 *
 * @return     the pid of the mqtt_control thread
 */
kernel_pid_t  mqtt_thread_init(char *stack, int stacksize, char priority, const char *name, void *arg);

/**
 * @brief      publish to a topic
 *
 * @param      pub_topic  The pub topic
 * @param      data       The data
 *
 * @return     status
 */
static int mqtt_pub(char* addr, char* port);
/**
 * @brief      authomatically subscribes to a mqtt topic
 *
 * @param      sub_topic  The sub topic
 *
 * @return     status
 */
static int mqtt_sub(char* sub_topic);


#endif /*  _MQTT_THREAD_H */ 
#ifndef _MQTT_H_
#define _MQTT_H_
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>


/**
 * This structure is used for locally storing the mqtt topic to id mapping
 */
typedef struct mqtt_topic_entry {
    struct mqtt_topic_entry *next;
    uint16_t id;
    char topic[16];
} mqtt_topic_entry_t;

/**
 * @brief      this function registers a new publishing topic id
 *
 * @param      entry  This argument contains the mapping of topic name to id
 */
void mqtt_topic_register(mqtt_topic_entry_t *entry);

/**
 * @brief      this function unregisters a new publishing topic id
 *
 * @param      entry  This argument contains the mapping of topic name to id
 */
void mqtt_topic_unregister(mqtt_topic_entry_t *entry);

/**
 * @brief      this function search for a topic id
 *
 * @param      topic  the topic name
 */
uint16_t mqtt_search_scalar (char topic[]);

#endif /* _MQTT_H_ */
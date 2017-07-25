#include "mqtt.h"
#include "utlist.h"

/**
 * Head of the mqtt topic link list
 */
static mqtt_topic_entry_t *mqtt_reg;

/**
 * @brief      this function registers a new publishing topic id
 *
 * @param      entry  This argument contains the mapping of topic name to id
 */
void mqtt_topic_register(mqtt_topic_entry_t *entry)
{
    LL_PREPEND(mqtt_reg, entry);
}


/**
 * @brief      this function unregisters a new publishing topic id
 *
 * @param      entry  This argument contains the mapping of topic name to id
 */
void mqtt_topic_unregister(mqtt_topic_entry_t *entry)
{
    LL_DELETE(mqtt_reg, entry);
}


/**
 * @brief      this function search for a topic id
 *
 * @param      topic  the topic name
 */
uint16_t mqtt_search_scalar (char topic[])
{
    mqtt_topic_entry_t *el;
    for(el = mqtt_reg ; el ; el=(el)->next)
    {
        // DEBUG("%s  ::  %s\n", el->topic, topic);
        if (strcmp(el->topic,topic) == 0)
        {
            return (el->id);
        }
    }
    return 0;
}

/*---------------------------------------------------------------------------*/
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/*
Requirement
===========

The task is to create a bisic contiki setup which collects the signal strength,
calculates the average RSSI for a sampling period and sends the data over UDP
to a specific address/port

The code should include the following:
* start a new contiki process
* call 'NETSTACK_RADIO.get_value(RADIO_PARAM_RSSI, &value)' in every second
* continuously calcualte the average RSSI for last minute, 5 minutes, 15 minutes
* every 30 seconds send the average values to the address "fd00::1",
* the values should be sent as ASCII separated by comma
* values should look like '-71,-80,-76'
*/

/*---------------------------------------------------------------------------*/
// Header files
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "contiki.h"
#include "contiki-lib.h"
#include "contiki-net.h"

/*---------------------------------------------------------------------------*/
// MACROs
#if APP_DEBUG_ENABLE
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define UDP_CLIENT_PORT             3001
#define UDP_SERVER_PORT             3000

#define UDP_SEND_INTERVAL           30 * CLOCK_SECOND   
#define RSSI_AVG_CAL_INTERVAL       1 * CLOCK_SECOND

#define ONE_MIN_COUNT               60
#define FIVE_MIN_COUNT              300
#define FIFTEEN_MIN_COUNT           900

#define BASE_DECIMAL                10

/*---------------------------------------------------------------------------*/
// Variable declarations
static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;

static struct ctimer rssi_avg_cal_ctimer;

char udp_data_buf[48] = {0, };

// for storing rssi value in every one second
int8_t fifteen_min_rssi_arr[FIFTEEN_MIN_COUNT] = {0, };
uint16_t arr_index = 0;

// for storing rssi average value
int8_t last_one_min_rssi_avg = 0;
int8_t last_five_min_rssi_avg = 0;
int8_t last_fifteen_min_rssi_avg = 0;

/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);

/*---------------------------------------------------------------------------*/
static int8_t 
calculate_avg(int8_t *data_buf, uint16_t data_len)
{
    int8_t average = 0;
    uint16_t i = 0;
    int sum = 0;

    for(i=0; i<data_len; i++)
    {
        sum += data_buf[i];
    }

    average = (sum / data_len);

    return average;
}

/*---------------------------------------------------------------------------*/
static void
handle_rssi_avg_cal(void *d)
{
    int rssi_val = 0;
    
    NETSTACK_RADIO.get_value(RADIO_PARAM_RSSI, &rssi_val);
    
    PRINTF("%d : ctimer expired : %i\r\n", arr_index, rssi_val);

    fifteen_min_rssi_arr[arr_index++] = (int8_t)rssi_val;

    if ((arr_index % ONE_MIN_COUNT) == 0)
    {
        last_one_min_rssi_avg = calculate_avg(&fifteen_min_rssi_arr[arr_index - ONE_MIN_COUNT], ONE_MIN_COUNT);
        PRINTF("last_one_min_rssi_avg : %i\r\n", last_one_min_rssi_avg);
    }

    if ((arr_index % FIVE_MIN_COUNT) == 0)
    {
        last_five_min_rssi_avg = calculate_avg(&fifteen_min_rssi_arr[arr_index - FIVE_MIN_COUNT], FIVE_MIN_COUNT);
        PRINTF("last_five_min_rssi_avg : %i\r\n", last_five_min_rssi_avg);
    }

    if (arr_index == FIFTEEN_MIN_COUNT)
    {
        last_fifteen_min_rssi_avg = calculate_avg(fifteen_min_rssi_arr, FIFTEEN_MIN_COUNT);
        PRINTF("last_fifteen_min_rssi_avg : %i\r\n", last_fifteen_min_rssi_avg);
        arr_index = 0;
    }

    ctimer_reset(&rssi_avg_cal_ctimer);
}

/*---------------------------------------------------------------------------*/
static void
set_server_address(void)
{
    uip_lladdr_t uip_lladdr;

    uip_ip6addr(&server_ipaddr, UIP_DS6_DEFAULT_PREFIX, 0, 0, 0, 0, 0, 0, 1);
    uip_ds6_addr_add(&server_ipaddr, 0, ADDR_AUTOCONF);
    uip_ds6_nbr_add(&server_ipaddr, &uip_lladdr, 0, NBR_REACHABLE, NBR_TABLE_REASON_ROUTE, NULL);
    uip_ds6_route_add(&server_ipaddr, 128, &server_ipaddr);
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
    static struct etimer udp_data_send_etimer;
    uip_ipaddr_t ipaddr;

    #if 0
    char buffer_1[16] = {0, };
    char buffer_2[16] = {0, };
    char buffer_3[16] = {0, };
    #endif

    // UDP client process started
    PRINTF("UDP client process started\r\n");
    PROCESS_BEGIN();

    set_server_address();

    // new connection with remote host
    client_conn = udp_new(&ipaddr, UIP_HTONS(UDP_SERVER_PORT), NULL);
    
    if(client_conn == NULL) {
        // no UDP connection available, exiting the process
        PROCESS_EXIT();
    }

    udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

    ctimer_set(&rssi_avg_cal_ctimer, RSSI_AVG_CAL_INTERVAL, handle_rssi_avg_cal, NULL);
    etimer_set(&udp_data_send_etimer, UDP_SEND_INTERVAL);

    while (1)
    {
        PROCESS_YIELD();

        if (etimer_expired(&udp_data_send_etimer))
        {
            #if 0
            // integer to ascii conversion with comma separated data
            itoa(last_one_min_rssi_avg, buffer_1, BASE_DECIMAL);
            itoa(last_five_min_rssi_avg, buffer_2, BASE_DECIMAL);
            itoa(last_fifteen_min_rssi_avg, buffer_3, BASE_DECIMAL);

            sprintf(udp_data_buf, "%s,%s,%s", buffer_1, buffer_2, buffer_3);
            #endif

            sprintf(udp_data_buf, "%i,%i,%i", last_one_min_rssi_avg, 
                                            last_five_min_rssi_avg, 
                                            last_fifteen_min_rssi_avg);

            PRINTF("udp_data_buf: %s\r\n", udp_data_buf);

            // rssi value sending to server
            uip_udp_packet_sendto(client_conn, (char*)udp_data_buf, strlen(udp_data_buf), 
                                    &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));

            etimer_restart(&udp_data_send_etimer);
        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/

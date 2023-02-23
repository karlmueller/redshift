#pragma once
#include <WiFi.h>

//still need to figure out how the hell to initiate esp now and send paclets


uint8_t broadcast_id[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


typedef struct short_packet {
    int sender_id;
    int recipient_id = broadcast_id;

}, sender_packet_T;

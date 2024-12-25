#include "mtf02p.h"

MTF02P *MTF02PSingleton::mtf02p = NULL;

bool MTF02P::micolink_decode(uint8_t data)
{
    if (micolink_parse_char(&msg, data) == false)
    {
        return false;
    }

    switch (msg.msg_id)
    {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
            memcpy(&payload, msg.payload, msg.len);
            /*
                You can get the sensor data here:

                distance           = payload.distance;
                distance strength  = payload.strength;
                distance precision = payload.precision;
                distance status    = payload.tof_status;
                flow velocity x    = payload.flow_vel_x;
                flow velocity y    = payload.flow_vel_y;
                flow quality       = payload.flow_quality;
                flow status        = payload.flow_status;
            */
            break;
        }

        default:
            break;
        }

    return true;
}

bool MTF02P::micolink_check_sum(MICOLINK_MSG_t *msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for (uint8_t i = 0; i < length; i++)
    {
        checksum += temp[i];
    }

    if (checksum == msg->checksum)
        return true;
    else
        return false;
}

bool MTF02P::micolink_parse_char(MICOLINK_MSG_t *msg, uint8_t data)
{
    switch (msg->status)
    {
    case 0:
        if (data == MICOLINK_MSG_HEAD)
        {
            msg->head = data;
            msg->status++;
        }
        break;

    case 1: // device id
        msg->dev_id = data;
        msg->status++;
        break;

    case 2: // system id
        msg->sys_id = data;
        msg->status++;
        break;

    case 3: // message id
        msg->msg_id = data;
        msg->status++;
        break;

    case 4: //
        msg->seq = data;
        msg->status++;
        break;

    case 5: // payload length
        msg->len = data;
        if (msg->len == 0)
            msg->status += 2;
        else if (msg->len > MICOLINK_MAX_PAYLOAD_LEN)
            msg->status = 0;
        else
            msg->status++;
        break;

    case 6: // payload receive
        msg->payload[msg->payload_cnt++] = data;
        if (msg->payload_cnt == msg->len)
        {
            msg->payload_cnt = 0;
            msg->status++;
        }
        break;

    case 7: // check sum
        msg->checksum = data;
        msg->status = 0;
        if (micolink_check_sum(msg))
        {
            return true;
        }

    default:
        msg->status = 0;
        msg->payload_cnt = 0;
        break;
    }

    return false;
}

void MTF02P::update()
{
    while (Serial2.available())
    {
        uint8_t c = Serial2.read();
        if (micolink_decode(c))
        {
            break;
        }
    }
}

float MTF02P::calibrateX(float gY)
{
    countX ++;
    currentCalibrationSumX += gY != 0 ? payload.flow_vel_x / -gY : 0;
    return currentCalibrationSumX / countX;
}

float MTF02P::calibrateY(float gX)
{
    countY ++;
    currentCalibrationSumY += gX != 0 ? payload.flow_vel_y / gX : 0;
    return currentCalibrationSumY / countY;
}

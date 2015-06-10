/*

  ImageSectionPacket, ImageTagPacket, and ImagePacketQueue
  derived from TelemetryPacket and ByteStringQueue

*/

#ifndef _IMAGE_HPP_
#define _IMAGE_HPP_

#include <vector>

#include "Telemetry.hpp"

#define SECTION_MAX_PIXELS 1300

class ImagePacket : public TelemetryPacket {
public:
    ImagePacket(uint8_t camera, uint16_t counter, Clock systemTime,
                uint16_t rows, uint16_t columns,
                uint16_t row_index, uint16_t col_index);

    //Use this constructor when needing to have an empty image section packet
    //Pass in NULL
    //This packet is non-functional!  Be sure not to use without reassignment!
    ImagePacket(const void *ptr);

    uint8_t getCamera();
    uint16_t getNumRows();
    uint16_t getNumColumns();
    uint16_t getRowIndex();
    uint16_t getColumnIndex();
    uint16_t getNumPixels();

    void finalize();
};

class ImagePacketQueue : public TelemetryPacketQueue {
private:
    uint16_t counter;

public:
    ImagePacketQueue();

    // row_end and column_end are exclusive ends, not inclusive ends
    void add_partial_array(uint8_t camera,
                           uint16_t rows, uint16_t columns,
                           uint16_t row_start, uint16_t row_end,
                           uint16_t column_start, uint16_t column_end,
                           const uint8_t *array, Clock systemTime,
                           bool last);

    void reassembleTo(uint8_t &camera, uint16_t &rows, uint16_t &columns,
                      std::vector<uint8_t> &output);
};

#endif

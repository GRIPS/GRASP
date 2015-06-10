#include <sys/time.h>
#include <string.h>
#include <math.h>

//#include <fstream>
#include <iostream>

#include "Image.hpp"
//#include "types.hpp" //for bitread and bitwrite

#define SYS_ID_PYC 0x4A
#define SYS_ID_RC 0x4B

#define TM_IMAGE 0x20

#define INDEX_NUM_ROWS 16
#define INDEX_NUM_COLUMNS 18
#define INDEX_ROW_INDEX 20
#define INDEX_COLUMN_INDEX 22
#define INDEX_PIXEL_DATA 24

using std::ostream;

class ImagePacketInvalidException : public std::exception
{
    virtual const char* what() const throw()
        {
            return "Invalid ImagePacket";
        }
} ipInvalidException;

class ImageQueueEmptyException : public std::exception
{
    virtual const char* what() const throw()
        {
            return "Empty ImagePacketQueue";
        }
} iqEmptyException;

ImagePacket::ImagePacket(uint8_t camera, uint16_t counter, Clock systemTime,
                         uint16_t rows, uint16_t columns,
                         uint16_t row_index, uint16_t column_index)
    : TelemetryPacket(camera == 0 ? SYS_ID_PYC : SYS_ID_RC,
                      TM_IMAGE, counter, systemTime)
{
    *this << rows << columns << row_index << column_index;
}

ImagePacket::ImagePacket(const void *ptr)
    : TelemetryPacket(ptr)
{
    //Assumes that NULL was passed in
}

void ImagePacket::finalize()
{
    TelemetryPacket::finish();
}

uint8_t ImagePacket::getCamera()
{
    uint8_t tm_type = this->getTmType();
    switch(tm_type) {
        case 0x4A:
            return 0;
        case 0x4B:
            return 1;
        default:
            return 0xFF;
    }
}

uint16_t ImagePacket::getNumRows()
{
    uint16_t num_rows;
    this->readAtTo(INDEX_NUM_ROWS, num_rows);
    return num_rows;
}

uint16_t ImagePacket::getNumColumns()
{
    uint16_t num_columns;
    this->readAtTo(INDEX_NUM_COLUMNS, num_columns);
    return num_columns;
}

uint16_t ImagePacket::getRowIndex()
{
    uint16_t row_index;
    this->readAtTo(INDEX_ROW_INDEX, row_index);
    return row_index;
}

uint16_t ImagePacket::getColumnIndex()
{
    uint16_t column_index;
    this->readAtTo(INDEX_COLUMN_INDEX, column_index);
    return column_index;
}

uint16_t ImagePacket::getNumPixels()
{
    return this->getLength() - INDEX_PIXEL_DATA;
}

ImagePacketQueue::ImagePacketQueue()
{
    counter = 0;
}

void ImagePacketQueue::add_partial_array(uint8_t camera,
                                         uint16_t rows, uint16_t columns,
                                         uint16_t row_start, uint16_t row_end,
                                         uint16_t column_start, uint16_t column_end,
                                         const uint8_t *array, Clock systemTime,
                                         bool last)
{
    ImagePacket ip(NULL);

    for (uint16_t i = row_start; i < row_end; i++) {
        ip = ImagePacket(camera, counter++, systemTime,
                         rows, columns, i, column_start);
        ip.append_bytes(array + i * columns + column_start, column_end - column_start);
        ip.finalize();
        *this << ip;
    }

    if (last) {
        ip = ImagePacket(camera, counter, systemTime,
                         rows, columns, rows, columns);
        ip.finalize();
        *this << ip;
        counter = 0;
    }
}

void ImagePacketQueue::reassembleTo(uint8_t &camera,
                                    uint16_t &rows, uint16_t &columns,
                                    std::vector<uint8_t> &output)
{
    ImagePacket ip(NULL);

    //Look for the first ImagePacket
    do {
        *this >> ip;
        if (!ip.valid()) throw ipInvalidException;
    } while (ip.getTmType() != TM_IMAGE);

    camera = ip.getCamera();
    rows = ip.getNumRows();
    columns = ip.getNumColumns();

    output.clear();
    output.resize(rows*columns);

    while(ip.getNumPixels() > 0) {
        if (ip.getTmType() != TM_IMAGE) break;
        uint16_t row_index = ip.getRowIndex();
        uint16_t column_index = ip.getColumnIndex();
        ip.readAtTo_bytes(INDEX_PIXEL_DATA,
                          &output[row_index * columns + column_index],
                          ip.getNumPixels());
        *this >> ip;
        if (!ip.valid()) throw ipInvalidException;
    }
}

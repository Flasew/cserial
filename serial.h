/*
 * File: serial.h
 * Author: Weiyang Wang <wew168@ucsd.edu>
 * Date: Oct 31, 2017
 * Description: A program for c serial I/O with object-oriented design.
 */

#ifndef SERIAL_H
#define SERIAL_H

#define TRUE 1
#define FALSE 0

/* struct including all data needed for the serial ports
 * currently not support flow control options
 */
typedef struct serial {
    int fd;                         // file descriptor
    int timeout;                    // read/write timeout in 1/10 seconds, 
                                    // -1 for infinite
    unsigned int baudrate;          // baud-rate
    char mode[4];                   // specifies databits, parity and stopbits
                                    // in an "DPS" format (ex. 8N1)
    char * port;                    // path to the device file
} serial;

/* array index for mode */
#define DATA_I 0                    // Data bit
#define PAR_I  1                    // parity bit
#define STOP_I 2                    // stop bit

/* functions. For convenience headers are included. */

/**
 * unsigned int serial_init(serial ** spp)
 *
 * Initialize a serial object on the heap. The initialized object will be 
 * assigned to the argument. 
 * Notice that serial_init and serial_destroy comes in pair, and any init call
 * MUST have a corresponding destroy call to prevent memory leak.
 *
 * Arguments:
 *  serial ** spp - pointer to a serial pointer. After function run *spp 
 *                  should give a pointer to a serial object on the heap
 *                  *spp MUST be NULL for the function to proceed.
 *
 * Returns:
 *  TRUE indicating success, FALSE otherwise. Errors are usually due to 
 *  heap memory allocation failure.
 */
unsigned int serial_init(serial ** spp);

/**
 * unsigned int serial_open(serial * serp, char * port, 
 *      int baudrate, double timeout, char * mode);
 *
 * Open an allocated serial port. ALL attributes need to be explicitly specified
 * since this is not python...
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object, to be opened.
 *  char * port - string describing path to the device file
 *  int baudrate - desired baudrate
 *  double timeout - desired read-timeout. -1 for blocking indefinitely, 0 for
 *                   non-blocking XXX: this might also set write timeout
 *                   with in the struct serial, this number is converted into
 *                   an int measured in 0.1 seconds
 *  char * mode - serial port data format, in a "DPS" format. See struct 
 *                deceleration for more details
 *
 * Returns:
 *  TRUE indicating success, FALSE otherwise. 
 */
unsigned int serial_open(serial * serp, char * port, 
    int baudrate, double timeout, char * mode);

/**
 * void serial_close(serial * serp);
 *
 * Close an allocated serial port. Must be called before destructor. 
 * (Destructor will actually check if it's closed so no worry...)
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object, to be closed.
 */
void serial_close(serial * serp);

/**
 * void serial_destroy(serial ** spp);
 *
 * Destroy a serial port allocated on the heap (free it). Will check if it's 
 * closed. MUST be called if init is used
 *
 * Arguments:
 *  serial ** spp - serial pointer pointer so that after free, the corresponding
 *                  serial * can be set to NULL.
 */
void serial_destroy(serial ** spp);

/**
 * unsigned int serial_set_baudrate(serial * serp, int baudrate);
 *
 * Set the baudrate of a serial port. The port need to be opened. 
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object
 *  int baudrate - desired baudrate
 * Returns:
 *  TRUE indicating success, FALSE otherwise. 
 */
unsigned int serial_set_baudrate(serial * serp, int baudrate);

/**
 * unsigned int serial_set_timeout(serial * serp, double timeout);
 *
 * set the timeout of a serial port. The port need to be opened. 
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object
 *  double timeout - desired read-timeout. -1 for blocking indefinitely, 0 for
 *                   non-blocking XXX: this might also set write timeout
 *                   with in the struct serial, this number is converted into
 *                   an int measured in 0.1 seconds
 * Returns:
 *  TRUE indicating success, FALSE otherwise. 
 */
unsigned int serial_set_timeout(serial * serp, double timeout);

/**
 * unsigned int serial_set_mode(serial * serp, char * mode);
 *
 * set the mode of a serial port. The port need to be opened. 
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object
 *  char * mode - serial port data format, in a "DPS" format. See struct 
 *                deceleration for more details
 * Returns:
 *  TRUE indicating success, FALSE otherwise. 
 */
unsigned int serial_set_mode(serial * serp, char * mode);

/**
 * int serial_read(serial * serp, char * buff, int n);
 *
 * Read at most @n bytes into @buff. By defut a read will end when an CR is hit.
 * Depending on the timeout setting, this call may block indefinitely. 
 * A null char will be appended at the end of buff after reading.
 * Notice that overflow is not checked therefore @buff could be overflowed, 
 * so it's better to make sure that n < buffsize - 1
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object
 *  char * buff - buffer to be read into
 *  int n - max bytes to read
 * 
 * Returns:
 *  Positive number for number of bytes read, negative number for error.
 */
int serial_read(serial * serp, char * buff, int n);

/**
 * int serial_read_until(serial * serp, char * buff, char end, int n);
 *
 * Read at most @n bytes into @buff or until @end is hit.
 * Depending on the timeout setting, this call may block indefinitely. 
 * A null char will be appended at the end of buff after reading.
 * Notice that overflow is not checked therefore @buff could be overflowed, 
 * so it's better to make sure that n < buffsize - 1
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object
 *  char * buff - buffer to be read into
 *  char end - end character of the read
 *  int n - max bytes to read
 * 
 * Returns:
 *  Positive number for number of bytes read, negative number for error.
 */
int serial_read_until(serial * serp, char * buff, char end, int n);

/**
 * int serial_write(serial * serp, char * msg);
 *
 * write @msg to the serial port. @msg is expected to be a null terminated 
 * string; when write is executed the end '\0' will be replaced by '\r' 
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object
 *  char * msg - null terminated string to be write 
 * 
 * Returns:
 *  Positive number for number of bytes written, negative number for error.
 */
int serial_write(serial * serp, char * msg);

/**
 * int serial_nwrite(serial * serp, char * msg, int n);
 *
 * write @n byte of @msg to the serial port. This could be more useful then
 * serial_write() since many device actually uses different indicator of end
 * of message, but care needs to be taken to make sure n = len(msg) otherwise
 * undefined behavior will occur.
 *
 * Arguments:
 *  serial * serp - pointer to an allocated serial object
 *  char * msg - string to be write 
 *  int n - number of bytes to be send
 * 
 * Returns:
 *  Positive number for number of bytes read, negative number for error.
 */
int serial_nwrite(serial * serp, char * msg, int n);

/* check if a serial port is open. Unopened serial ports should have @fd filed
   be -1 EXCEPT those just allocated on the stack, which need special care.*/
unsigned int serial_is_open(serial * serp);

#endif
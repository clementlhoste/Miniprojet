#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

//Constants for the image processing 
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MAX_DISTANCE			25
#define MIN_LINE_WIDTH			40
#define PXTOCM					1570.0f

/*
Exports the result of the standard deviation (-)
performed on the last picture
 */
float get_std_dev(void);


/*
Exports the value (px) of the center of the line
detected in the last picture
 */
uint16_t get_line_position(void);

/*
Exports the width (px) of the last line
detected in the last picture
 */
//uint16_t get_line_width(void);

/*
Starts the Process Image Thread
Mainly line recognition
Standard Deviation calculation
 */
void process_image_start(void);


#endif /* PROCESS_IMAGE_H */

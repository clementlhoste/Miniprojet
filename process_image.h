#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

//Constant for the image processing
#define IMAGE_BUFFER_SIZE		640

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
Starts the Process Image Thread
Mainly line recognition
Standard Deviation calculation
 */
void process_image_start(void);


#endif /* PROCESS_IMAGE_H */

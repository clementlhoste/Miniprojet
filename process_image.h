#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

#define MSK_GREEN1 0b00000111
#define	MSK_GREEN2 0b11100000

/*
 */
uint16_t get_line_width(void);

/*
 */
float get_std_dev(void);

/*
 */
uint16_t get_line_position(void);

/*
 */
void process_image_start(void);


#endif /* PROCESS_IMAGE_H */

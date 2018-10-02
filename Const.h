/*
 * Const.h
 */

#ifndef CONST_H_
#define CONST_H_

#define STOP_DRIVE              (0x42)//'B'
#define SOFT_STOP               (0x41)//'A'
#define DRIVE_FORWARD           (0x43)//'C'
#define DRIVE_BACKWARD          (0x44)//'D'
#define DRIVE_FORWARD_DISTANCE  (0x45)//'E'
#define DRIVE_BACKWARD_DISTANCE (0x46)//'F'
#define INCREASE_SPEED          (0x47)//'G'
#define DECREASE_SPEED          (0x48)//'H'
#define DRIVE_FORWARD_10CM      (0x49)//'I'
#define DRIVE_BACKWARD_10CM     (0x4A)//'J'


#define SERIAL_ID               '2'//(0x32)
#define SERIAL_ID_SLAVE         (0x31)//"1"
#define STATE_OFF       ((int)0)
#define STATE_ON        ((int)1)
#define STATE_TOGGLE    ((int)2)

#define FORWARD         ((int)1)
#define BACKWARD        ((int)0)

#define UP              ((int)1)
#define DOWN            ((int)0)

#define MODE_1          ((int)0)
#define MODE_2          ((int)1)

#define FAULT_MOTOR_1   ((int)1)
#define FAULT_MOTOR_3   ((int)2)

#define TRANSITION      ((int)0)
#define BOTTOM          ((int)1)
#define TOP             ((int)2)




#endif /* CONST_H_ */

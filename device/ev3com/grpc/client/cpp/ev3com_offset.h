#ifndef _EV3COM_OFFSET_H_
#define _EV3COM_OFFSET_H_

/*
 * RX
 */
/*
 * HEADER
 */
#define EV3_SENSOR_HEADER_OFF         (0U)
#define EV3_SENSOR_VERSION_OFF        (4U)
#define EV3_SENSOR_SIMTIME_OFF        (16U)
#define EV3_SENSOR_EXTOFF_OFF         (24U)
#define EV3_SENSOR_EXTSIZE_OFF        (28U)

/*
 * BODY
 */
#define EV3_SENSOR_BODY_OFF           (32U)
/*
 * Button
 */
#define EV3_GPIO_BTN_OFF	(0U)
#define EV3_GPIO_BTN_BITS_LEFT	(0U)
#define EV3_GPIO_BTN_BITS_RIGHT	(1U)
#define EV3_GPIO_BTN_BITS_UP	(2U)
#define EV3_GPIO_BTN_BITS_DOWN	(3U)
#define EV3_GPIO_BTN_BITS_ENTER	(4U)
#define EV3_GPIO_BTN_BITS_BACK	(5U)

/*
 * Sensor
 */
#define EV3_SENSOR_OFF				(4U)
#define EV3_SENSOR_OFF_TYPE(index)	(EV3_SENSOR_OFF + ((index) * 4U))
#define EV3_SENSOR_INX_AMBIENT		(0U)
#define EV3_SENSOR_INX_COLOR0		(1U)
#define EV3_SENSOR_INX_REFLECT0		(2U)
#define EV3_SENSOR_INX_RGB_R0		(3U)
#define EV3_SENSOR_INX_RGB_G0		(4U)
#define EV3_SENSOR_INX_RGB_B0		(5U)
#define EV3_SENSOR_INX_ANGLE		(6U)
#define EV3_SENSOR_INX_RATE			(7U)
#define EV3_SENSOR_INX_IR_D			(8U)
#define EV3_SENSOR_INX_IR_0			(9U)
#define EV3_SENSOR_INX_IR_0_H		(10U)
#define EV3_SENSOR_INX_IR_0_D		(11U)
#define EV3_SENSOR_INX_IR_1			(12U)
#define EV3_SENSOR_INX_IR_1_H		(13U)
#define EV3_SENSOR_INX_IR_1_D		(14U)
#define EV3_SENSOR_INX_IR_2			(15U)
#define EV3_SENSOR_INX_IR_2_H		(16U)
#define EV3_SENSOR_INX_IR_2_D		(17U)
#define EV3_SENSOR_INX_IR_3			(18U)
#define EV3_SENSOR_INX_IR_3_H		(19U)
#define EV3_SENSOR_INX_IR_3_D		(20U)
#define EV3_SENSOR_INX_ULTRASONIC	(21U)
#define EV3_SENSOR_INX_ULTRASONIC_LISTEN	(22U)
#define EV3_SENSOR_INX_AXES_X		(23U)
#define EV3_SENSOR_INX_AXES_Y		(24U)
#define EV3_SENSOR_INX_AXES_Z		(25U)
#define EV3_SENSOR_INX_TEMP			(26U)
#define EV3_SENSOR_INX_TOUCH_0		(27U)
#define EV3_BATTERY_INX_CURRENT		(28U)
#define EV3_BATTERY_INX_VOLTAGE		(29U)
#define EV3_SENSOR_INX_TOUCH_1		(30U)
/*
 * COLOR2(RX)
 */
#define EV3_SENSOR_INX_COLOR1		(31U)
#define EV3_SENSOR_INX_REFLECT1		(32U)
#define EV3_SENSOR_INX_RGB_R1		(33U)
#define EV3_SENSOR_INX_RGB_G1		(34U)
#define EV3_SENSOR_INX_RGB_B1		(35U)


/*
 * MOTOR(RX)
 */
#define EV3_SENSOR_MOTOR_OFF				(256U)
#define EV3_SENSOR_MOTOR_OFF_TYPE(index)	(EV3_SENSOR_MOTOR_OFF + ((index) * 4U))
#define EV3_SENSOR_MOTOR_INX_ANGLE_TOP	(0U)
#define EV3_SENSOR_MOTOR_INX_ANGLE_A	(0U)
#define EV3_SENSOR_MOTOR_INX_ANGLE_B	(1U)
#define EV3_SENSOR_MOTOR_INX_ANGLE_C	(2U)
#define EV3_SENSOR_MOTOR_INX_ANGLE_D	(3U)

/*
 * GPS(RX)
 */
#define EV3_SENSOR_GPS_LAT_OFF          (480U)
#define EV3_SENSOR_GPS_LON_OFF          (488U)

/*
 * TX
 */
/*
 * HEADER
 */
#define EV3_ACTUATOR_HEADER_OFF         (0U)
#define EV3_ACTUATOR_VERSION_OFF        (4U)
#define EV3_ACTUATOR_SIMTIME_OFF        (8U)
#define EV3_ACTUATOR_EXTOFF_OFF         (24U)
#define EV3_ACTUATOR_EXTSIZE_OFF        (28U)

/*
 * BODY
 */
#define EV3_ACTUATOR_BODY_OFF           (32U)

/*
 * LED
 */
#define EV3_GPIO_LED_OFF	(0U)
#define EV3_GPIO_LED_BITS_RED		(0U)
#define EV3_GPIO_LED_BITS_GREEN		(1U)
#define EV3_GPIO_LED_BITS_YELLOW	(2U)
#define EV3_GPIO_LED_BITS_BLUE		(3U)

/*
 * MOTOR(TX)
 */
#define EV3_MOTOR_OFF				(4U)
#define EV3_MOTOR_OFF_TYPE(index)	(EV3_MOTOR_OFF + ((index) * 4U))
#define EV3_MOTOR_INX_POWER_TOP		(0U)
#define EV3_MOTOR_INX_POWER_A		(0U)
#define EV3_MOTOR_INX_POWER_B		(1U)
#define EV3_MOTOR_INX_POWER_C		(2U)
#define EV3_MOTOR_INX_POWER_D		(3U)

#define EV3_MOTOR_INX_STOP_TOP		(4U)
#define EV3_MOTOR_INX_STOP_A		(4U)
#define EV3_MOTOR_INX_STOP_B		(5U)
#define EV3_MOTOR_INX_STOP_C		(6U)
#define EV3_MOTOR_INX_STOP_D		(7U)

#define EV3_MOTOR_INX_RESET_ANGLE_TOP	(8U)
#define EV3_MOTOR_INX_RESET_ANGLE_A		(8U)
#define EV3_MOTOR_INX_RESET_ANGLE_B		(9U)
#define EV3_MOTOR_INX_RESET_ANGLE_C		(10U)
#define EV3_MOTOR_INX_RESET_ANGLE_D		(12U)

#define EV3_GYRO_OFF                (52U)
#define EV3_GYRO_OFF_TYPE(index)           (EV3_GYRO_OFF + ((index) * 4U))
#define EV3_GYRO_INX_RESET_TOP      (0U)
#define EV3_GYRO_INX_RESET          (0U)

#endif /* _EV3COM_OFFSET_H_ */

byte DRIVE=0
byte SAFE=1
byte FULL=2
byte POWER=3
byte SPOT=4
byte CLEAN=5
byte MAX=6
byte MOTORS=7
byte FORCE_SEEK_DOCK=8
byte WAKEUP=9
byte MOTORS_OFF=10
byte DRIVE_DIRECT=11
byte DOCK=12
byte DRIVE_PWM=13
byte DRIVE_FB=14
byte SONG=15


int32 DEFAULT_VELOCITY=200
int32 STRAIGHT_RADIUS=32768
byte TURN_CLOCK=-1
byte TURN_CNT_CLOCK=1
int32 mode
int32 velocity
int32 radius
geometry_msgs/Twist cntl
int16 r_pwm
int16 l_pwm

uint8[] song_melody

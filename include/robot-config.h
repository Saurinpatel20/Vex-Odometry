using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor RightFront;
extern motor RightBack;
extern motor LeftFront;
extern motor LeftBack;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Authors:      Cameron Barclay, Jayden Liffick, Teo Carrion              */
/*    Created:      12/6/2023, 10:45:31 PM                                    */
/*    Description:  Main code for team 8823Z robot.                           */
/*                                                                            */
/*----------------------------------------------------------------------------*/

//$ To keep things clean, declare functions at the top and write what they do at the bottom
#include <functional>

// Include library files
#include "../evAPI/evAPIFiles.h"

// Select namespaces ------------------------------------------------------
// using namespace vex;
// using namespace evAPI;

// Setup global objects ---------------------------------------------------
evAPI::Drive driveBase = evAPI::Drive(evAPI::blueGearBox);
evAPI::DriverBaseControl driveControl = evAPI::DriverBaseControl(&primaryController, evAPI::RCControl, &driveBase);
evAPI::vexUI UI;

// Setup vex component objects (motors, sensors, etc.) --------------------
auto leftEncoder = vex::rotation(vex::PORT14);
auto rightEncoder = vex::rotation(15);
auto centerEncoder = vex::rotation(16);

auto intakeMotor = vex::motor(vex::PORT5, vex::gearSetting::ratio6_1, true);

auto triport = vex::triport(vex::PORT22);
auto backLatch = vex::digital_out(triport.A);

auto intakeSensor = vex::optical(vex::PORT3);

//Setup Auto Button UI IDs
enum autoOptions {
  //*General
  //Page 1
  AUTO_DO_NOTHING = 0,
  AUTO_RIGHT,
  AUTO_LEFT,
};

//Variables to display on the controller
uint32_t batteryLevel = Brain.Battery.capacity();
int speed = 0;
std::string selectedAutoName = "";

//Setup controller UI IDs
enum controllerOptions
{
  //Match Screen
  MATCH_SCREEN = 0,

  //Disabled / Auto Screen
  DISABLED_AUTO_SCREEN = 3,
  AUTO_MODE_LABEL,
  AUTO_MODE_TEXT,

  //Inertial Calibration Screen
  INERTIAL_CALIBRATE_SCREEN = 6,
  INERTIAL_CALIBRATING_TEXT
};

/*---------------------------------------------------------------------------------*/
/*                             Pre-Autonomous Functions                            */
/*---------------------------------------------------------------------------------*/
void pre_auton(void) {
  //* Setup for auto selection UI ============================================
  // Add all the buttons
  UI.autoSelectorUI.addButton(AUTO_DO_NOTHING, vex::red);
  UI.autoSelectorUI.addButton(AUTO_RIGHT, vex::green);
  UI.autoSelectorUI.addButton(AUTO_LEFT, vex::blue);

  // Set all the titles
  UI.autoSelectorUI.setButtonTitle(AUTO_DO_NOTHING, "DO NOTHING!");
  UI.autoSelectorUI.setButtonTitle(AUTO_RIGHT, "Right Side Auton");
  UI.autoSelectorUI.setButtonTitle(AUTO_LEFT, "Left Side Auton");

  // Set all the descriptions
  UI.autoSelectorUI.setButtonDescription(AUTO_DO_NOTHING, "The robot will do nothing.");
  UI.autoSelectorUI.setButtonDescription(AUTO_RIGHT, "Right Side Auton");
  UI.autoSelectorUI.setButtonDescription(AUTO_LEFT, "Left Side Auton");

  // Select all the icons
  UI.autoSelectorUI.setButtonIcon(AUTO_DO_NOTHING, UI.autoSelectorUI.icons.exclamationMark);
  UI.autoSelectorUI.setButtonIcon(AUTO_RIGHT, UI.autoSelectorUI.icons.rightArrow);
  UI.autoSelectorUI.setButtonIcon(AUTO_LEFT, UI.autoSelectorUI.icons.leftArrow);

  //Setup parameters for auto selector
  UI.autoSelectorUI.setSelectedButton(AUTO_LEFT);
  UI.autoSelectorUI.setSelectedPage(0);
  UI.autoSelectorUI.setDataDisplayTime(1500);

  //*Setup controller UI
  //Driver Control Screen
  UI.primaryControllerUI.addData(MATCH_SCREEN, "Battery: ", batteryLevel);
  UI.primaryControllerUI.addData(MATCH_SCREEN, "Speed : ", speed);

  //Disabled Screen
  UI.primaryControllerUI.addData(DISABLED_AUTO_SCREEN, "Battery: ", batteryLevel);
  UI.primaryControllerUI.addData(AUTO_MODE_LABEL, "Selected Auto:");
  UI.primaryControllerUI.addData(AUTO_MODE_TEXT, "", selectedAutoName);

  //Calibrating Inertial Screen
  UI.primaryControllerUI.addData(INERTIAL_CALIBRATE_SCREEN, "Battery: ", batteryLevel);
  UI.primaryControllerUI.addData(INERTIAL_CALIBRATING_TEXT, "Calibrating...");

  //*Secondary Controller 
  UI.secondaryControllerUI.addData(0, "Battery: ", batteryLevel);

  //Start the threads
  UI.startThreads();

  //* Setup for smart drive ==================================================
  driveBase.setDebugState(true);

  // Setup motor settings
  driveBase.leftPortSetup(11, 12, 13, 15);
  driveBase.rightPortSetup(7, 8, 9, 10);
  driveBase.leftReverseSetup(true, true, true, true);
  driveBase.rightReverseSetup(false, false, false, false);
  driveBase.geartrainSetup(3.25, 24, 36);
  driveBase.setDriveBaseWidth(13);
  
  // Setup inertial sensor settings
  driveBase.setupInertialSensor(4);

  // Set default speeds
  driveBase.setDriveSpeed(100);
  driveBase.setTurnSpeed(100);
  driveBase.setArcTurnSpeed(40);

  //Set stopping mode
  driveBase.setStoppingMode(vex::brake);

  // Setup PID
  // driveBase.setupDrivePID(0.125, 10, 0.005, 12, 2, 125);
  driveBase.setupDrivePID(0.125, 4, 0.05, 12, 2, 125);
  driveBase.setupDriftPID(0.015, 0, 0, 1, 0, 0);
  // driveBase.setupTurnPID(0.65, 0, .65, 3, 1, 100);
  driveBase.setupTurnPID(0.6, 1, .65, 2, 1, 100);
  driveBase.setupArcPID(0.1, 5, 0, 3, 2, 200);
  driveBase.setupArcDriftPID(0.2, 0, 0, 1, 0, 0);

  //* Setup for base driver contorl ==========================================
  driveControl.setPrimaryStick(evAPI::leftStick);
  driveControl.setHandicaps(1, 0.6);  // main drive, turning

  //* Setup controller callbacks =============================================
  // Example:
  // primaryController.LEFT_WINGS_BUTTON.pressed(toggleLeftWing);
  primaryController.ButtonR1.pressed([](){
    static bool spinning = false;
    if (spinning) {
      intakeMotor.stop();
      spinning = false;
    } else {
      intakeMotor.spin(vex::directionType::fwd, 100.0, vex::percentUnits::pct);
      spinning = true;
    }
  });

  primaryController.ButtonL1.pressed([](){
    backLatch.set(!backLatch.value());
  });

  //*Display calibrating and autonomous information if connected to a field or comp switch
  if(evAPI::isConnectToField()) UI.primaryControllerUI.setScreenLine(INERTIAL_CALIBRATE_SCREEN);

  //Calibrate inertial.
  printf("Calibrating...\n");
  driveBase.calibrateInertial();
  while(driveBase.isInertialCalibrating())
  {
    vex::this_thread::sleep_for(5);
  }
  printf("Calibrated\n");
  if(evAPI::isConnectToField()) UI.primaryControllerUI.setScreenLine(DISABLED_AUTO_SCREEN);

  intakeSensor.setLight(vex::ledState::on);
  intakeSensor.setLightPower(75);
}

void waitRingIntaked() {
    while (true) {
      vex::color detectedColor = intakeSensor.color();
      if (detectedColor == vex::color::red || detectedColor == vex::color::blue) {
        break;
      }
    }
}

template<typename T>
void onRingIntaked(T callback) {
  auto fn = [](void* callback) {
    waitRingIntaked();
    (*static_cast<T*>(callback))();
  };

  vex::thread(fn, &callback).detach();
}

/*---------------------------------------------------------------------------------*/
/*                                 Autonomous Task                                 */
/*---------------------------------------------------------------------------------*/
void autonomous(void) {
  //Times how long auto takes
  vex::timer autoTimer;

  switch (UI.autoSelectorUI.getSelectedButton()) {
    case AUTO_RIGHT:
      driveBase.driveBackward(14.5, 50);
      driveBase.driveBackward(10, 30);

      backLatch.set(true);

      intakeMotor.spin(vex::directionType::fwd, 40, vex::pct);
      vex::this_thread::sleep_for(3000);
      intakeMotor.stop();

      driveBase.turnToHeading(-88);
      intakeMotor.spin(vex::directionType::fwd, 100, vex::pct);
      driveBase.driveForward(18, 40);

      backLatch.set(false);
      vex::this_thread::sleep_for(1500);
      backLatch.set(true);
      vex::this_thread::sleep_for(500);

      waitRingIntaked();
      intakeMotor.spin(vex::directionType::fwd, 40, vex::pct);

      driveBase.turnToHeading(-180);

      backLatch.set(false);
      vex::this_thread::sleep_for(1500);
      backLatch.set(true);
      vex::this_thread::sleep_for(500);
      break;

    case AUTO_LEFT: {
      bool go = false;
      driveBase.driveBackward(14.5, 50);
      driveBase.driveBackward(10, 30);

      backLatch.set(true);
      
      intakeMotor.spin(vex::directionType::fwd, 40, vex::pct);
      vex::this_thread::sleep_for(3000);
      intakeMotor.stop();

      driveBase.turnToHeading(88);
      intakeMotor.spin(vex::directionType::fwd, 100, vex::pct);
      driveBase.driveForward(20, 40);

      onRingIntaked([&](){
        intakeMotor.spin(vex::directionType::fwd, 40, vex::pct);
        vex::this_thread::sleep_for(3000);
        go = true;
      });

      backLatch.set(false);
      vex::this_thread::sleep_for(1500);
      backLatch.set(true);

      while (true) {
        if (go) {
          break;
        }
      }
      
      driveBase.turnToHeading(180);

      backLatch.set(false);
      vex::this_thread::sleep_for(1500);
      backLatch.set(true);
      vex::this_thread::sleep_for(500);
      break;
    }
    //*Do nothing auto
    case AUTO_DO_NOTHING:
      //!DO NOTHING HERE
      break;

    //*Default auto that runs if an unknown ID is selected
    default:
      printf("Invalid Auto Selected.\n");
      primaryController.rumble("---");
      break;
  }

  //Print out how long the auto took
  printf("Auto Time: %f\n", autoTimer.value());
}

/*---------------------------------------------------------------------------------*/
/*                                 User Control Task                               */
/*---------------------------------------------------------------------------------*/

double changeToDistance(double change, double radius) {
  return (change * (radius * M_PI)) / (2 * M_PI);
}

void usercontrol(void) {
  UI.primaryControllerUI.setScreenLine(MATCH_SCREEN);

  while (1) {
    //=========== All drivercontrol code goes between the lines ==============

    //* Control the base code -----------------------------
    driveControl.driverLoop();

    vex::color detectedColor = intakeSensor.color();
    if (detectedColor == vex::color::red || detectedColor == vex::color::blue) {
      vex::thread([](){
        intakeMotor.setVelocity(40, vex::percentUnits::pct);
        vex::this_thread::sleep_for(1500);
        intakeMotor.setVelocity(100, vex::percentUnits::pct);
      }).detach();
    }
    
    

    //========================================================================
    //* Control the intake code ---------------------------

    vex::task::sleep(20); // Sleep the task for a short amount of time to prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  
  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    //*Update controller UI data
    batteryLevel = Brain.Battery.capacity();
    speed = (driveBase.getMotorSpeed(vex::left) + driveBase.getMotorSpeed(vex::right)) / 2;

    //Update auto name
    selectedAutoName = UI.autoSelectorUI.getSelectedButtonTitle();

    vex::task::sleep(20);
  }
}

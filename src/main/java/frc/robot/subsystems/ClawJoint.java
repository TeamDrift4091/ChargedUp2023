package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ClawJoint{

  private WPI_TalonFX motor; // motor is an object of the WPI_TalonFX class that represents the motor. We will use this object to control the motor, such as setting its speed or position.
  private AbsoluteEncoder encoder; // encoder is an object of the AbsoluteEncoder class that represents the encoder. We will use this object to read the position and velocity of the joint.
  private double positionConversionFactor; // positionConversionFactor is a double variable that we will use to convert the encoder's position to an angle in degrees. We will set this variable in the constructor of our class.
  private double velocityConversionFactor; //velocityConversionFactor is a double variable that we will use to convert the encoder's velocity to an angular velocity in degrees per second. We will set this variable in the constructor of our class.
  
  public ClawJoint(int motorId, int encoderId){ // This is the constructor of our class. It takes two arguments: motorId and encoderId, which are integers representing the IDs of the motor and encoder.

    motor = new WPI_TalonFX(motorId); //  creates a new instance of the WPI_TalonFX class and assigns it to the motor variable.
    encoder = new AbsoluteEncoder(encoderId); //creates a new instance of the AbsoluteEncoder class and assigns it to the encoder variable
    // Set the position and velocity conversion factors
    positionConversionFactor = 360.0 / encoder.getAbsolutePositionRange(); //calculates the conversion factor from encoder units to degrees. We divide 360.0 by the absolute position range of the encoder, which gives us the number of degrees per encoder unit.
    velocityConversionFactor = 360.0 / encoder.getAbsolutePositionRange() * 60.0 / encoder.getVelocityConversionFactor(); //calculates the conversion factor from encoder units per second to degrees per second. We divide 360.0 by the absolute position range of the encoder to get the number of degrees per encoder unit, then multiply by 60.0 to convert from minutes to seconds, and divide by the velocity conversion factor of the encoder to get the number of degrees per encoder unit per second.
  }

  public void setAngle(double angle){
    //impliment method to drive motor to target angle
  }
  public double getAngle() {
    // Get the absolute position from the encoder and convert it to an angle
    double absolutePosition = encoder.getAbsolutePosition(); // The getAbsolutePosition() method reads information from the encoder and tells the robot the exact position of the joint. The absolutePosition variable stores that information.
    double angle = absolutePosition * positionConversionFactor; // The positionConversionFactor is a scaling factor that helps convert the encoder reading into an angle in degrees. The angle variable stores the angle of the joint, which helps the robot understand where the joint is located. 
    return angle; //The method returns the angle as a double.
}
public double getAngularVelocity() {
  // Get the velocity from the encoder and convert it to an angular velocity
  double velocity = encoder.getVelocity(); // The getVelocity() method reads the velocity of the joint from the encoder and stores it in the velocity variable.
  double angularVelocity = velocity * velocityConversionFactor; //The velocityConversionFactor is a scaling factor that helps convert the velocity measurement from the encoder into degrees per second. The angularVelocity variable stores the angular velocity of the joint, which helps the robot understand how fast the joint is moving. 
  return angularVelocity; // The method returns the angular velocity as a double.
    } 
  }

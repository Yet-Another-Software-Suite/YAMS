package yams;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Optional;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.thethriftybot.devices.ThriftyNova;

import edu.wpi.first.math.system.plant.DCMotor;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorFactory;
import yams.units.CRTAbsoluteEncoder;
import yams.units.CRTAbsoluteEncoderConfig;

public class SmartMotorFactoryTest {

  @BeforeAll
  static void setupMockFactories() {
    SmartMotorFactory.availableControllers.put("com.ctre.phoenix6.hardware.TalonFX",
        (params) -> mock(SmartMotorController.class));
    SmartMotorFactory.availableControllers.put("com.ctre.phoenix6.hardware.TalonFXS",
        (params) -> mock(SmartMotorController.class));
    SmartMotorFactory.availableControllers.put("com.revrobotics.spark.SparkBase",
        (params) -> mock(SmartMotorController.class));
    SmartMotorFactory.availableControllers.put("com.thethriftybot.ThriftyNova",
        (params) -> mock(SmartMotorController.class));
  }

  @Test
  void testFactoryRegistration() {
    assertFalse(SmartMotorFactory.availableControllers.isEmpty(), "Factory should have registered controllers");
  }

  @Test
  void testCreateTalonFXWrapper() {
    TalonFX mockController = mock(TalonFX.class);
    DCMotor mockMotor = createMockDCMotor();
    SmartMotorControllerConfig mockConfig = createMockSmartConfig();

    Optional<SmartMotorController> result = SmartMotorFactory.create(mockController, mockMotor, mockConfig);

    assertTrue(result.isPresent(), "Factory should create a TalonFX SmartMotorController");
  }

  @Test
  void testCreateTalonFXSWrapper() {
    TalonFXS mockController = mock(TalonFXS.class);
    DCMotor mockMotor = createMockDCMotor();
    SmartMotorControllerConfig mockConfig = createMockSmartConfig();

    Optional<SmartMotorController> result = SmartMotorFactory.create(mockController, mockMotor, mockConfig);

    assertTrue(result.isPresent(), "Factory should create a TalonFXS SmartMotorController");
  }

  @Test
  void testCreateSparkWrapper() {
    DCMotor mockMotor = createMockDCMotor();

    Optional<SmartMotorController> result = SmartMotorFactory.create(
        new SparkMax(1, MotorType.kBrushed),
        mockMotor,
        new SmartMotorControllerConfig(null));

    assertTrue(result.isPresent(), "Factory should create a Spark SmartMotorController");
  }

  @Test
  void testCRT()
  {
    var primeGear1 = 13;
    var primeGear2 = 23;
    var config = new CRTAbsoluteEncoderConfig(()->Rotations.of(0.15),()->Rotations.of(0.05))
        .withAbsoluteEncoder1Gearing(20,32,primeGear1)
        .withAbsoluteEncoder2Gearing(20,32,primeGear2);
    var encoder = new CRTAbsoluteEncoder(config);
    System.out.println("CRT Angle(rots): "+encoder.getAngle().in(Rotations));
    System.out.println("CRT Angle(degrees): "+encoder.getAngle().in(Degrees));

    // TODO: Implement the reverse CRT here

    assertEquals(encoder.getAngle(), Rotations.of(.71875));

  }

//  @Test
//  void testCreateNovaWrapper() {
//    ThriftyNova mockController = mock(ThriftyNova.class);
//    DCMotor mockMotor = createMockDCMotor();
//    SmartMotorControllerConfig mockConfig = createMockSmartConfig();
//
//    Optional<SmartMotorController> result = SmartMotorFactory.create(mockController, mockMotor, mockConfig);
//
//    assertTrue(result.isPresent(), "Factory should create a Nova SmartMotorController");
//  }

  @Test
  void testCreateUnsupportedClassReturnsEmpty() {
    class DummyController {
    }

    DummyController dummy = new DummyController();

    Optional<SmartMotorController> result = SmartMotorFactory.create(dummy, null, null);

    assertFalse(result.isPresent(), "Factory should return empty for unsupported classes");
  }

  private static DCMotor createMockDCMotor() {
    return DCMotor.getBag(1);
  }

  private static SmartMotorControllerConfig createMockSmartConfig() {
    return new SmartMotorControllerConfig(null);
  }
}

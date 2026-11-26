package frc.robot.subsystems.tracao;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.campo.Colisoes;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;
import org.littletonrobotics.junction.Logger;

/** Tração tank (4 CIM), só simulação, com colisão contra o campo. */
public class Tracao extends SubsystemBase {
  private final DifferentialDrivetrainSim sim =
      new DifferentialDrivetrainSim(
          DCMotor.getCIM(2),
          10.71, // redução da Toughbox Mini: PREENCHER
          7.5, // inércia (kg·m²): PREENCHER
          50.0, // massa (kg): PREENCHER
          Units.inchesToMeters(3.0), // raio da roda
          0.66, // distância entre as rodas dos dois lados: PREENCHER
          null);

  private double esquerdaVolts = 0.0;
  private double direitaVolts = 0.0;

  // Diz se o braço fica sem colisão com o robô nesta posição do campo.
  private Predicate<Pose2d> bracoLivre = pose -> true;

  public void setBracoLivre(Predicate<Pose2d> bracoLivre) {
    this.bracoLivre = bracoLivre;
  }

  /** Coloca o robô parado em um ponto do campo. */
  public void colocar(Pose2d pose) {
    sim.setState(
        VecBuilder.fill(
            pose.getX(), pose.getY(), pose.getRotation().getRadians(), 0, 0, 0, 0));
  }

  public Tracao() {
    sim.setPose(new Pose2d(2.0, 2.0, new Rotation2d()));
  }

  public Pose2d getPose() {
    return sim.getPose();
  }

  /** Um valor para andar e outro para girar (positivo = anti-horário). */
  public Command arcade(DoubleSupplier frente, DoubleSupplier giro) {
    return run(
        () -> {
          var rodas =
              DifferentialDrive.arcadeDriveIK(
                  MathUtil.applyDeadband(frente.getAsDouble(), 0.08),
                  MathUtil.applyDeadband(giro.getAsDouble(), 0.08),
                  true);
          esquerdaVolts = rodas.left * 12.0;
          direitaVolts = rodas.right * 12.0;
        });
  }

  @Override
  public void periodic() {
    boolean parado = DriverStation.isDisabled();
    sim.setInputs(parado ? 0.0 : esquerdaVolts, parado ? 0.0 : direitaVolts);
    Pose2d antes = sim.getPose();
    sim.update(0.02);

    // 1. Chassi: empurra para fora de paredes e obstáculos (desliza neles).
    Pose2d depois = sim.getPose();
    Pose2d corrigida = Colisoes.chassiNoCampo(depois);
    boolean bateu = corrigida.getTranslation().getDistance(depois.getTranslation()) > 1e-6;
    if (bateu) {
      sim.setPose(corrigida);
    }
    // 2. Braço: se ele bateria em algo, o robô não sai do lugar.
    boolean bracoBateu = !bracoLivre.test(corrigida) && bracoLivre.test(antes);
    if (bracoBateu) {
      colocar(antes);
    }
    Logger.recordOutput("Campo/ChassiBateu", bateu);
    Logger.recordOutput("Campo/BracoBateu", bracoBateu);

    Logger.recordOutput("Robo/Pose", sim.getPose());
  }
}

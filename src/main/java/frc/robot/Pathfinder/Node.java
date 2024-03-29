package frc.robot.Pathfinder;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Node {
  double x, y;
  Rotation2d holonomicRotation;
  List<Node> neighbors;

  public Node(double x, double y) {
    this.x = x;
    this.y = y;
    this.holonomicRotation = Rotation2d.fromDegrees(0);
    this.neighbors = new ArrayList<>();
  }

  public Node(double x, double y, Rotation2d holonomicRotation) {
    this.x = x;
    this.y = y;
    this.holonomicRotation = holonomicRotation;
    this.neighbors = new ArrayList<>();
  }

  public Node(Translation2d coordinates, Rotation2d holonmRotation2d) {
    this.x = coordinates.getX();
    this.y = coordinates.getY();
    this.holonomicRotation = holonmRotation2d;
    this.neighbors = new ArrayList<>();
  }

  public Node(Pose2d pose) {
    this.x = pose.getX();
    this.y = pose.getY();
    this.holonomicRotation = pose.getRotation();
    this.neighbors = new ArrayList<>();
  }

  public void addNeighbor(Node neighbor) {
    this.neighbors.add(neighbor);
  }

  public double getX() {
    return x;
  }

  public double getY() {
    return y;
  }

  public Rotation2d getHolRot() {
    return holonomicRotation;
  }

  public void setHolRot(double degree) {
    this.holonomicRotation = Rotation2d.fromDegrees(degree);
  }

  @Override
  public String toString() {
    return "X Position: " + getX() + "\t" + "Y Position: " + getY();
  }
}

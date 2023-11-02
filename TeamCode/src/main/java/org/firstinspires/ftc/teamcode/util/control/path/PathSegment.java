// package org.firstinspires.ftc.teamcode.util.control.path;
//
// interface Function {
//     double f(double t);
// }
// class PathFunction {
//     Function x;
//     Function y;
//     double pathLength = 0.0;
//     static final double dt = 0.01;
//
//     public PathFunction(Function x, Function y) {
//         this.x = x;
//         this.y = y;
//         pathLength = generatePathLength();
//     }
//
//     public Point f(double t) {
//         return new Point(x.f(t), y.f(t));
//     }
//
//     public double cost(Point p, double t) {
//         return Math.pow(x.f(t), 2) + Math.pow(y.f(t), 2);
//     }
//
//     private double generatePathLength() {
//         for (int i = 0; i < 1/dt; i++) {
//             // ruh roh
//             pathLength += dsAt((double) i/(1/dt)) * dt;
//         }
//         return pathLength;
//     }
//
//     public double dsAt(double t) {
//         return f(t).displacement(f(t+dt));
//     }
//
//     public Vector2d tangentAt(double t) {
//         return new Vector2d(f(t+dt).x - f(t).x, f(t+dt).y - f(t).y);
//     }
//
//     public double newton(Point p, double guess) {
//         double oldGuess = guess;
//         for (int i = 0; i < 10; i++) {
//             guess = oldGuess - (cost(p, oldGuess)
//                     / (cost(p, oldGuess) - cost(p, oldGuess+dt)));
//             if (Math.abs(guess - oldGuess) < 0.1) {
//                 break;
//             }
//             oldGuess = guess;
//         }
//         return guess;
//     }
// }
//
// public class PathSegment {
//     PathFunction f;
//     // step one half tile's length at a time: it is very unlikely for there to be 2 local minima in
//     // this range
//     static final double STEP_LENGTH = 12;
//
//     public PathSegment(PathFunction f) {
//         this.f = f;
//     }
//     // always return a normalized vector
//     // i don't think this is actually necessary given we have getEndPose()
// //    public Vector2d getEndVector();
//     public Point getClosestPoint(Point p) {
//         return f.f(getClosestT(p));
//     };
//     public Pose2d getEndPose() {
//         return new Pose2d(f.f(1), f.tangentAt(1));
//     };
//     public Vector2d getTau(Point p) {
//         return f.tangentAt(getClosestT(p));
//     };
//
//     public double getClosestT(Point p) {
//         double lowestCost = Double.MAX_VALUE;
//         double closestT = 0.0;
//         for (double i = 0; i < f.pathLength; i += STEP_LENGTH/f.pathLength) {
//             if (f.cost(p, i) < lowestCost) {
//                 // repeated operation, maybe remove
//                 lowestCost = f.cost(p, i);
//                 closestT = i;
//             }
//         }
//         return f.newton(p, closestT);
//     }
//
// }
//
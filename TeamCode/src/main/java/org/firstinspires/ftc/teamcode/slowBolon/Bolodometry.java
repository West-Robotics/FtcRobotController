package org.firstinspires.ftc.teamcode.slowBolon;

public class Bolodometry {
    double x,y, angle;
    double last_x, last_y, last_angle;
    double fl, fr, bl, br;
    double lfl, lfr, lbl, lbr;
    double[] wheelmove;
    double[][] blwheelmove, brwheelmove, flwheelmove, frwheelmove;
    public void resetposition() {
        x = 0; y = 0; angle = 0;
    }
    public void updateposition(double infr, double infl, double inbr, double inbl, double inangle) {

        fl = infl; fr = infr; bl = inbl; br = inbr;
/*
        double forward = (fr-lfr) + (fl-lfl) + (br-lbr) + (bl-lbl);
        double side = -(fr-lfr) + (fl-lfl) - (bl-lbl) + (br-lbr);

        y += forward;
        x += side;*/

        // "c" -- change in encoder value
        double blc = (bl-lbl), brc = br-lbr, flc = fl-lfl, frc = fr-lfr;

        // total vector of motion should equal the sum of the wheels
        double[][] allwheelmove = new double[2][2];
        for(int x = 0; x < blwheelmove.length; x++) {
            for(int y = 0; y < blwheelmove[0].length; y++) {
                allwheelmove[x][y] = (blwheelmove[x][y]*blc) + (brwheelmove[x][y]*brc) + (flwheelmove[x][y]*flc) + (frwheelmove[x][y]*frc);
            }
        }

        // adjust for the angle the robot is facing and log changes
        allwheelmove = multiplyMatrices(allwheelmove,getRotationMatrixForAngle(inangle));
        x += allwheelmove[0][0];
        y += allwheelmove[0][1];

        last_x = x; last_y = y; last_angle = angle;
        lbr = br; lbl = bl;
        lfr = fr; lfl = fl;
    }

    double[][] multiplyMatrices(double[] firstMatrix, double[][] secondMatrix) {
        double[][] result = new double[firstMatrix.length][secondMatrix[0].length];

        for (int row = 0; row < result.length; row++) {
            for (int col = 0; col < result[row].length; col++) {
                result[row][col] = multiplyMatricesCell(firstMatrix, secondMatrix, row, col);
            }
        }

        return result;
    }
    double multiplyMatricesCell(double[] firstMatrix, double[][] secondMatrix, int row, int col) {
        double cell = 0;
        for (int i = 0; i < secondMatrix.length; i++) {
            cell += firstMatrix[i] * secondMatrix[i][col];
        }
        return cell;
    }
    double[][] multiplyMatrices(double[][] firstMatrix, double[][] secondMatrix) {
        double[][] result = new double[firstMatrix.length][secondMatrix[0].length];

        for (int row = 0; row < result.length; row++) {
            for (int col = 0; col < result[row].length; col++) {
                result[row][col] = multiplyMatricesCell(firstMatrix, secondMatrix, row, col);
            }
        }

        return result;
    }
    double multiplyMatricesCell(double[][] firstMatrix, double[][] secondMatrix, int row, int col) {
        double cell = 0;
        for (int i = 0; i < secondMatrix.length; i++) {
            cell += firstMatrix[row][i] * secondMatrix[i][col];
        }
        return cell;
    }

    public double[][] getRotationMatrixForAngle(double rotateangle) {
        // INPUT IN DEGREES, TO BE TRANSLATED TO RADIANS -- VERY IMPORTANT
        rotateangle = rotateangle *  Math.PI/180;
        return new double[][] {
                {
                        Math.cos(rotateangle), -Math.sin(rotateangle)
                },
                {
                        Math.sin(rotateangle), Math.cos(rotateangle)
                }
        };
    }

    public void init() {
        resetposition();

        // get base vector of motion for each wheel
        wheelmove = new double[] {0,1};
        blwheelmove = multiplyMatrices(wheelmove,getRotationMatrixForAngle(315));
        brwheelmove = multiplyMatrices(wheelmove,getRotationMatrixForAngle(45));
        flwheelmove = multiplyMatrices(wheelmove,getRotationMatrixForAngle(45));
        frwheelmove = multiplyMatrices(wheelmove,getRotationMatrixForAngle(315));
    }
}

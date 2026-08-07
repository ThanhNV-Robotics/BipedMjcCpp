#include "data_logger.h"
#include "joystick_interpreter.h"
#include "Eigen/Dense"

int main (int argc, const char** argv)
{
    const double dt = 0.02;
    const double vx = 0.7; //reference base velocity
    const double startWalkingTime = 2;
    const double simuDuration = 6; 

    const double initial_base_x_pos = 0;
    const double initial_base_y_pos = 0;

    double t{0};
    
    JoyStickInterpreter jsInterpreter (dt);
    DataLogger logger("../record/test_interpreter.log"); // data logger

    logger.addIterm("simTime", 1);
    logger.addIterm("vx_L", 1);
    logger.addIterm("vy_L", 1);
    logger.addIterm("wz_L", 1);
    logger.addIterm("thetaZ", 1);
    logger.addIterm("vx_W", 1);
    logger.addIterm("vy_W", 1);
    logger.addIterm("px_W", 1);
    logger.addIterm("py_W", 1);
    logger.finishItermAdding();

    while (t < simuDuration)
    {
        t += dt;
        if ( t >= startWalkingTime)
        {
            jsInterpreter.setWzDesLPara(0, 1);
            jsInterpreter.setVxDesLPara(vx, 2.0); // jsInterp.setVxDesLPara(0.9,1);
        }
        else {
            jsInterpreter.setIniPos(initial_base_x_pos,initial_base_y_pos, 0);
        }
        jsInterpreter.step();

        logger.startNewLine();
        logger.recItermData("simTime", t);
        logger.recItermData("vx_L", jsInterpreter.vx_L);
        logger.recItermData("vy_L", jsInterpreter.vy_L);
        logger.recItermData("wz_L", jsInterpreter.wz_L);
        logger.recItermData("thetaZ", jsInterpreter.thetaZ);
        logger.recItermData("vx_W", jsInterpreter.vx_W);
        logger.recItermData("vy_W", jsInterpreter.vy_W);
        logger.recItermData("px_W", jsInterpreter.px_W);
        logger.recItermData("py_W", jsInterpreter.py_W);
        logger.finishLine();
    }
    

    return 0;
}
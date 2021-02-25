#include "net/robocup_ssl_client.h"
#include "net/grSim_client.h"
#include "util/timer.h"
#include "util/util.h"

#include "header.h"
#include "path_planning.h"

double to180range(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle < -M_PI)
    {
        angle = angle + 2 * M_PI;
    }
    else if (angle > M_PI)
    {
        angle = angle - 2 * M_PI;
    }
    return angle;
}

double smallestAngleDiff(double target, double source)
{
    double a;
    a = fmod(target + 2 * M_PI, 2 * M_PI) 
        - fmod(source + 2 * M_PI, 2 * M_PI);

    if (a > M_PI)
    {
        a = a - 2 * M_PI;
    }
    else if (a < -M_PI)
    {
        a = a + 2 * M_PI;
    }
    return a;
}

void PID(bot_t robot, 
        objective_t objective, 
        int index, 
        bool my_robots_are_yellow, 
        GrSim_Client *grSim_client )
{
    double Kp = 20;
    double Kd = 2.5;
    static double lastError = 0;

    double rightMotorSpeed;
    double leftMotorSpeed;

    bool reversed = false;

    double angle_rob = robot.a;

    double angle_obj = atan2( objective.y - robot.y, 
                            objective.x - robot.x );

    double error = smallestAngleDiff(angle_rob, angle_obj);

    if (fabs(error) > M_PI / 2.0 + M_PI / 20.0)
    {
        reversed = true;
        angle_rob = to180range(angle_rob + M_PI);
        // Calculates the error and reverses the front of the robot
        error = smallestAngleDiff(angle_rob, angle_obj);
    }

    double motorSpeed = (Kp * error) + (Kd * (error - lastError)); 
    // + 0.2 * sumErr;
    
    lastError = error;

    double baseSpeed = 30;

    // Normalize
    motorSpeed = motorSpeed > 30 ? 30 : motorSpeed;
    motorSpeed = motorSpeed < -30 ? -30 : motorSpeed;

    // RESUMED PID THAT DOENST WORK
    // if (reversed) baseSpeed = -baseSpeed;

    // leftMotorSpeed = baseSpeed + motorSpeed / 2;
    // rightMotorSpeed = baseSpeed - motorSpeed / 2;
    // THIS SHIT CANT SPIN ON ITSELF

    if (motorSpeed > 0)
    {
        leftMotorSpeed = baseSpeed;
        rightMotorSpeed = baseSpeed - motorSpeed;
    }
    else
    {
        leftMotorSpeed = baseSpeed + motorSpeed;
        rightMotorSpeed = baseSpeed;
    }

    if (reversed)
    {
        if (motorSpeed > 0)
        {
            leftMotorSpeed = -baseSpeed + motorSpeed;
            rightMotorSpeed = -baseSpeed;
        }
        else
        {
            leftMotorSpeed = -baseSpeed;
            rightMotorSpeed = -baseSpeed - motorSpeed;
        }
    }

    // send the command
    grSim_client->sendCommand(leftMotorSpeed, 
                                rightMotorSpeed, 
                                my_robots_are_yellow, 
                                index);
}

void execute_bot_strats(field_t *f, GrSim_Client *commandClient)
{

    // for (int i = 0; i < NUM_BOTS; i++) {
    //     bot_t our_robot = f->our_bots[i];
    //
    //     vector<bot_t> other_robots;

    //     for (int i = 0; i < f->our_bots_n; i++)
    //     {
    //         if (i != our_robot.index)
    //             other_robots.push_back(f->our_bots[i]);
    //     }
    //     for (int i = 0; i < f->their_bots_n; i++)
    //         other_robots.push_back(f->their_bots[i]);

    //     if(!our_robot.wants_to_hit_ball)
    //     {
    //         bot_t ball_translated;
    //         ball_translated.x = f->ball.x;
    //         ball_translated.y = f->ball.y;
    //         ball_translated.radius = 5;
    //         other_robots.push_back(ball_translated);
    //     }

    //     objective_t o = path(other_robots, our_robot, 
    //                         our_robot.obj.x, our_robot.obj.y, our_robot.obj.angle);
    //     PID(our_robot, o, our_robot.index, 
    //         f->my_robots_are_yellow, commandClient);
    // }

    for (int i = 0; i < NUM_BOTS; i++){
        PID(f->our_bots[i], f->our_bots[i].obj, i, 
            f->my_robots_are_yellow, commandClient);
    }

}
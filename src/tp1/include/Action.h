#ifndef ACTION_H
#define ACTION_H

#include <vector>
#pragma once
const int kInputSize = 196;     
const int kHiddenSize = 128;
const int kOutputSize = 3;
enum MotionMode {MANUAL, WANDER, FARFROMWALLS};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, AUTO};

typedef struct
{
    MotionMode mode;
    MovingDirection direction;
} MotionControl;

class Action
{
public:
    Action();
    
    void manualRobotMotion(MovingDirection direction);
    void avoidObstacles(std::vector<float> speed);
    void keepAsFarthestAsPossibleFromWalls(std::vector<float> lasers, std::vector<float> sonars);

    MotionControl handlePressedKey(char key);

    void correctVelocitiesIfInvalid();
    float getLinearVelocity();
    float getAngularVelocity();

private:
    float linVel;
    float angVel;
};







#endif // ACTION_H

#include "Tuple.hpp"
#include <iostream>
#include <string>
#include <cmath>

using MazeCellPose = mtrn3100::Tuple<float, float, float>;

MazeCellPose* command2pose(std::string commands) {
    int length = commands.length() - 2;
    MazeCellPose* poseSequence = new MazeCellPose[length];

    poseSequence[0] = {(commands[0] - '0') * 250, (commands[1] - '0') * 250, 0};

    for (int i = 1; i <= length; ++i) {
        char command = commands[i + 2];
        float prev_pose[3] = {mtrn3100::get<0>(poseSequence[i - 1]), mtrn3100::get<1>(poseSequence[i - 1]), mtrn3100::get<2>(poseSequence[i - 1])};
        switch (command) {
            case 'F':
                poseSequence[i] = {prev_pose[0] + 250 * cos(prev_pose[2]), prev_pose[1] + 250 * sin(prev_pose[2]), prev_pose[2]};
                break;
            case 'R':
                poseSequence[i] = {prev_pose[0], prev_pose[1], prev_pose[2] + (M_PI / 2)};
                break;
            case 'L':
                poseSequence[i] = {prev_pose[0], prev_pose[1], prev_pose[2] - (M_PI / 2)};
                break;
        }
    }
    return poseSequence;
}
int main() {
    std::string commands = "11NFLFLFRRFRR";
    MazeCellPose* poseSequence = command2pose(commands);
    for (int i = 0; i < commands.length() - 2; ++i) {
        std::cout << mtrn3100::get<0>(poseSequence[i]) << ", "
                  << mtrn3100::get<1>(poseSequence[i]) << ", "
                  << mtrn3100::get<2>(poseSequence[i]) << std::endl;
    }

    // Don't forget to free the memory allocated for the poseSequence array
    delete[] poseSequence;

    return 0;
}

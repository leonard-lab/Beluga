function [SPD, OMEGA, ZDOT, IX] = belugaSetKinematicsIPC(robot_id, spd, omega, zdot, sock)

[~, ~, ~, SPD, OMEGA, ZDOT, IX] = belugaSetControlIPC(robot_id, [], [], [], spd, omega, zdot, sock);
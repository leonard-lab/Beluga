function [SPD, OMEGA, ZDOT, IX] = belugaGetKinematicsIPC(robot_id, sock)

[~, ~, ~, SPD, OMEGA, ZDOT, IX] = begluaGetControlIPC(robot_id, sock);
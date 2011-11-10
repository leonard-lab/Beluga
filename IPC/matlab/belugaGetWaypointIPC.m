function [X, Y, Z, IX] = belugaGetWaypointIPC(robot_id, sock)

[X, Y, Z, ~, ~, ~, IX] = belugaGetControlIPC(robot_id, sock);
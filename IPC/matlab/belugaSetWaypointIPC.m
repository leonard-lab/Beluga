function [X, Y, Z, IX] = belugaSetWaypointIPC(robot_id, x, y, z, sock)

[X, Y, Z, ~, ~, ~, IX] = belugaSetControlIPC(robot_id, x, y, z, [], [], [], sock);
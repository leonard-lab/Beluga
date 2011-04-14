<?php

/* beluga.php
 *
 * Dan Swain - dan.t.swain@gmail.com - 4/7/11
 *
 */

$server_address = '127.0.0.1';
$server_port = 1234;

require_once('../IPC/phpClient.php');
$client = new belugaClient();

if(!$client->connect($server_address, $server_port))
{
    trigger_error('Could not connect to IPC server.', E_USER_ERROR);
}

/* grab input variables */
$robot = (isset($_GET["robot"]) ? $_GET["robot"] : "");
$x = (isset($_GET["go_x"]) ? $_GET["go_x"] : "");
$y = (isset($_GET["go_y"]) ? $_GET["go_y"] : "");

if(!(strlen($robot) && strlen($x) && strlen($y)))
{
    trigger_error('Must specify robot, go_x, and go_y', E_USER_ERROR);
}

$resp1 = $client->sendMessage("set command " . $robot . " " . $x . " " . $y . " 0");
echo $client->sendMessage("get position 0");

?>
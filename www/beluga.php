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
    echo "<span>The IPC server is down.</span>";
    die();
}

/* grab input variables */
$robot = (isset($_GET["robot"]) ? $_GET["robot"] : "");
$x = (isset($_GET["go_x"]) ? $_GET["go_x"] : "");
$y = (isset($_GET["go_y"]) ? $_GET["go_y"] : "");
$z = (isset($_GET["go_z"]) ? $_GET["go_z"] : "");

$message = "";

if(!(strlen($robot) && strlen($x) && strlen($y)))
{
    $message = "";
    if(isset($_GET["get_commands"]))
    {
        $message = "get command *";
    }
    else
    {
        $message = "get position *";
    }
    echo $client->sendMessage($message);
}
else
{
    $message = "set command " . $robot . " " . $x . " " . $y . " ";
    if(strlen($z))
    {
        $message = $message . $z;
    }
    else
    {
        $message = $message . "0";
    }

    $resp1 = $client->sendMessage($message);
    echo $client->sendMessage("get position *");
}


?>

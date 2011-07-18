<?php

$server_address = '127.0.0.1';
$server_port = 1234;

require_once('../IPC/phpClient.php');
$client = new belugaClient();

if(!$client->connect($server_address, $server_port))
{
    header('Location:beluga_down.html');
}
else
{
    header('Location:beluga.html');
}

?>
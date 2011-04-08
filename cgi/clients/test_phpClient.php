<?php

require_once('phpClient.php');

$client = new belugaClient();

if(!$client->connect('127.0.0.1', 1234))
{
    printf("Error connecting.");
}
else
{
    printf("Connected.\n");
    printf($client->lastResponse()."\n");
}

echo $client->sendMessage("ping") . "\n";

echo $client->sendMessage("set position 1 -1 -2 -3") . "\n";
echo $client->sendMessage("get position *") . "\n";

?>
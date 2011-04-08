<?php

class belugaClient
{
    private $host;
    private $port;
    private $socket;
    private $timeout = 2;
    private $connected = false;
    private $error_file = "errors_php.log";
    private $last_response = "";
    
    function __construct()
    {
    }

    function __destruct()
    {
        if($connected)
        {
            fclose($socket);
        }
    }

    function connect($to_host, $on_port, $with_timeout = 2)
    {
        $this->timeout = $with_timeout;
        $this->host = $to_host;
        $this->port = $on_port;

        $this->socket = fsockopen($this->host,
                                   $this->port,
                                   $errnum,
                                   $errstr,
                                   $this->timeout);
        if(!is_resource($this->socket))
        {
            error_log($errstr, 3, $this->error_file);
            $this->connected = false;
        }
        else
        {
            $this->last_response = trim(fgets($this->socket));
            $this->connected = true;
        }
        return $this->connected;
    }

    function isConnected()
    {
        return $this->connected;
    }
    
    function lastResponse(){return $this->last_response;}

    function sendMessage($message)
    {
        if(!$this->connected)
        {
            return "Not connected";
        }
        
        $message = trim($message) . "\n";
        fputs($this->socket, $message);
        $this->last_response = trim(fgets($this->socket));
        return $this->last_response;
    }
    
}

?>
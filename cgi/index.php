<?php

if(file_exists("is_running"))
{
    header('Location:beluga.html');
}
else
{
    header('Location:beluga_down.html');
}

?>
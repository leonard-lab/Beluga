<?php

/* beluga.php
 *
 * Dan Swain - dan.t.swain@gmail.com - 3/31/11
 *
 * Simple cgi responder for the beluga project.
 * Receives robot index, x and y "go to" positions,
 * responds with current position of all robots
 *
 * Operates on a very simple principle of writing to one
 * file and reading from another.
 *
 * See README to make sure your server/permissions are
 * set up correctly.
 *
 */

/* configuration variables */
$cmd_file_fmt = "cmd_robot_%d";       // format for command file names
$position_file = "positions_to_cgi";  // file name that holds positions
$position_fmt = "%d ";                // printf-style format for output
$n_robots = 4;                        // number of robots being used

/* grab input variables */
$robot = (isset($_GET["robot"]) ? $_GET["robot"] : "");
$x = (isset($_GET["go_x"]) ? $_GET["go_x"] : "");
$y = (isset($_GET["go_y"]) ? $_GET["go_y"] : "");

/* if robot was not specified, we can't write a command */
if(strlen($robot))
{
    /* make sure we're accessing a valid robot index */
    if($robot < $n_robots)
    {
        $out_file = sprintf($cmd_file_fmt, $robot);

        /* make sure we have a command to write */
        if(strlen($x) && strlen($y))
        {
            /* write the output as two floats */
            $fp = fopen($out_file, "w");
            fprintf($fp, "%f %f\n", $x, $y);
            fclose($fp);        
        }
    }
}

/* make sure the position file exists */
if(file_exists($position_file))
{

    /* read in file and split on spaces into an array */
    $file_contents = trim(file_get_contents($position_file));
    $positions = explode(" ", $file_contents);

    /* print the results separated by spaces */
    foreach($positions as $position)
    {
        printf("%d ", $position);
    }
}




?>
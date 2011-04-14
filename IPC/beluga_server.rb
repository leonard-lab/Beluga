require File.dirname(__FILE__) + "/rhubarb/rhubarb.rb"

class Robot

  attr_accessor :cmd_x, :cmd_y, :cmd_z, :position_x, :position_y, :position_z, :robot_id
  
  def initialize(id)
    @cmd_x = 0;
    @cmd_y = 0;
    @cmd_z = 0;
     
    @position_x = 0;
    @position_y = 0;
    @position_z = 0;
     
    @robot_id = id;
  end

end

class BelugaServer < Rhubarb
  
  def initialize(*args)
    super(*args)

    # these are class variables because we want each client
    # to know about them
    @@robots = Array.new;
    @@robots << Robot.new(0);
    @@robots << Robot.new(1);
    @@robots << Robot.new(2);
    @@robots << Robot.new(3);

    cmd_position = {:name => "position", :setArgs => 3, :maxIndex => 3}
    cmd_cmd = {:name => "command", :setArgs => 3, :maxIndex => 3}

    @indexed_get_commands << cmd_position
    @indexed_get_commands << cmd_cmd
    @indexed_set_commands << cmd_position
    @indexed_set_commands << cmd_cmd

  end

  def robot(id)
    @@robots[id]
  end

  def getCommand(i)
    " #{robot(i).robot_id} #{robot(i).cmd_x} #{robot(i).cmd_y} #{robot(i).cmd_z}"
  end

  def setCommand(i, args)
    @@robots[i].cmd_x = args[0].to_f
    @@robots[i].cmd_y = args[1].to_f
    @@robots[i].cmd_z = args[2].to_f
    getCommand(i)
  end

  def getPosition(i)
    " #{robot(i).robot_id} #{robot(i).position_x} #{robot(i).position_y} #{robot(i).position_z}"
  end

  def setPosition(i, args)
    @@robots[i].position_x = args[0].to_f
    @@robots[i].position_y = args[1].to_f
    @@robots[i].position_z = args[2].to_f
    getPosition(i)
  end
  
  def respondToPing(words)
    "PONG!"
  end

  def welcomeMessage(args)
    "Welcome to BelugaServer, client #{@@client_id}."
  end

end

server = BelugaServer.new(1234, '127.0.0.1')

server.audit = true
server.start

server.join

puts "Server has been terminated"

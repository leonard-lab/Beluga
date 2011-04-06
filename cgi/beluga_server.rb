#!/usr/bin/ruby

require 'gserver'

Server_Shutdown = "S"
Server_ClientQuit = "Q"

Server_Cmds = [
               {:name => "get"},
               {:name => "set"},
               {:name => "ping"}
              ]

Cmd_Position = {:name =>"position", :setArgs =>3}
Cmd_Cmd = {:name => "command", :setArgs =>3}

Server_Cmds_GetIndexed = [Cmd_Position, Cmd_Cmd]
Server_Cmds_SetIndexed = [Cmd_Position, Cmd_Cmd]

Server_Cmds_Get = []
Server_Cmds_Set = []

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

class BelugaServer < GServer
  
  def initialize(*args)
    super(*args)

    # these are class variables because we want each client
    # to know about them
    @@robots = Array.new;
    @@robots << Robot.new(0);
    @@robots << Robot.new(1);
    @@robots << Robot.new(2);
    @@robots << Robot.new(3);
    @@index_max = 3;

    @@client_id = 0
    @@chat = []
    
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

  def respondToIndexedOp(words, op, resp_def)

    return unknownCommand({:message => "Not Enough Arguments"}) unless words.size >= 3

    opwords = words[2..words.size]
    responder = op + resp_def[:name].capitalize
    
    if respond_to?(responder)
      if op == "set"
        nargs = resp_def[:setArgs]
        return "Indexing error" unless opwords.size % (nargs + 1) == 0
        response = ""
        (0..opwords.size-nargs-1).step(nargs+1).each{|w|
          if opwords[w].to_i.to_s == opwords[w] && (0..@@index_max).include?(opwords[w].to_i)
            response += send(responder, opwords[w].to_i, opwords[w+1..w+nargs])
          end
        }
        response = "Indexing error" if response.strip.empty?
        return response
      elsif op == "get"
        do_all = opwords.include?("*")
        response = ""
        (0..@@index_max).each{|i|
          response += send(responder, i) if opwords.include?(i.to_s) or do_all
        }
        response = "Indexing error" if response.strip.empty?
        return response
      else
        unkownCommand({:message => "Unknown indexed command"})
      end
    else
      unknownCommand({:message => "No responder"})
    end

    unknownCommand({:message => "No responder present"})

  rescue
    unknownCommand({:message => "Responding to indexed command"})
  end

  def respondToSet(words)
    resp_def = Server_Cmds_SetIndexed.detect{|c| c[:name] == words[1]}

    if resp_def
      respondToIndexedOp(words, "set", resp_def)
    else
      resp_def = Server_Cmds_Set.detect{|c| c[:name] == words[1]}
      if resp_def && respond_to?("set" + words[1].capitalize)
        send("set" + words[1].capitalize, words, resp_def)
      else
        unknownCommand({:message => "Unknown Set command"})
      end
    end

  rescue unknownCommand({:message => "Responding to Set command"})
      
  end

  
  def respondToGet(words)
    resp_def = Server_Cmds_GetIndexed.detect{|c| c[:name] == words[1]}

    if resp_def
      respondToIndexedOp(words, "get", resp_def)
    else
      resp_def = Server_Cmds_Get.detect{|c| c[:name] == words[1]}
      if resp_def && respond_to?("get" + words[1].capitalize)
        send("get" + words[1].capitalize, words, resp_def)
      else
        unknownCommand({:message => "Unknown Get command"})
      end
    end

  rescue unknownCommand({:message => "Responding to Get command"})
      
  end

  def respondToPing(words)
    "PONG!"
  end

  def unknownCommand(def_info = {})
    info = {
      :method_name => "",
      :message => "",
      :self_message => ""
    }.merge def_info

    resp = "ERR " + self.class.to_s
    resp += " in method " + info[:method_name] unless info[:method_name].empty?
    resp += ": " + info[:message] unless info[:message].empty?

    in_resp = Time.now.strftime("[%a %b %e %R:%S %Y] ") + self.class.to_s
    in_resp += " " +  self.host.to_s + ":" + self.port.to_s + " ERR "
    in_resp += " in method " + info[:method_name] unless info[:method_name].empty?
    in_resp += ": " + info[:message] unless info[:message].empty?    
    in_resp += " + " + info[:self_message] unless info[:self_message].empty?
    puts in_resp

    resp
    
  end
  
  def serve(io)
    # Increment the client ID so each client gets a unique ID
    @@client_id += 1
    my_client_id = @@client_id

    io.puts("Hello from BelugaServer.  You are client #{@@client_id}.  I have 4 Robots.")

    do_shutdown = false
    
    loop do

      begin
        line = io.readline.strip

        if line == Server_ClientQuit
          io.puts "Goodbye"
          break
        end

        if line == Server_Shutdown
          io.puts "Goodbye"
          do_shutdown = true
          break
        end

        line = line.downcase
        words = line.split(" ")

        cmd_def = Server_Cmds.detect{|c| c[:name] == words[0]}
        responder = "respondTo" + words[0].capitalize

        if respond_to?(responder)
          io.puts send(responder, words).strip
        else
          io.puts unknownCommand({:message => "Unknown primary command"})
        end

      rescue
        io.puts "Server error.  Try again."
      end
      
    end

    if do_shutdown
      self.stop
    end
    
   end
end

server = BelugaServer.new(1234, '192.168.1.2')

server.audit = true
server.start

server.join

puts "Server has been terminated"

require 'spec_helper'

describe BelugaServer do

  before(:each) do
    BelugaServer.reset_data
    @server = BelugaServer.new(1234, '127.0.0.1')
    @server.start

    @sock = TCPSocket.open('127.0.0.1', 1234)
    @welcome = @sock.gets
  end

  it "should have the right welcome message" do
    @welcome.should match "Welcome to BelugaServer, client 1."
  end

  it "should respond to ping" do
    response_to("ping").should match "PONG!"
  end

  describe "positions" do

    it "should get positions" do
      (0..3).each do |i|
        response_to("get position #{i}").should match(("0.0 "*3).strip)
      end
    end

    it "should set positions (random data)" do
      (0..3).each do |i|
        p = Array.new(3).collect{ 3.0*(rand()-0.5) }
        p_as_str = p.join(" ")
        response_to("set position #{i} #{p_as_str}").should match "#{p_as_str}"
      end
    end

    # illustrative example
    it "should set positions" do
      response_to("set position 0 1.2 3.4 5.6").should match "1.2 3.4 5.6"
      response_to("get position 0").should match "1.2 3.4 5.6"
      
      response_to("set position 1 9.8 7.6 5.4").should match "9.8 7.6 5.4"
      response_to("get position 1").should match "9.8 7.6 5.4"

      response_to("set position 2 -1.0 -3.0 5.0").should match "-1.0 -3.0 5.0"
      response_to("get position 2").should match "-1.0 -3.0 5.0"

      response_to("set position 3 10.555 -0.001 1.0").should match "10.555 -0.001 1.0"
      response_to("get position 3").should match "10.555 -0.001 1.0"

      response_to("get position *").
        should match "1.2 3.4 5.6 9.8 7.6 5.4 -1.0 -3.0 5.0 10.555 -0.001 1.0"
    end

    it "should get all positions" do
      response_to("get position *").should match(("0.0 "*9).strip)
    end

    it "should set positions en masse (random data)" do
      p = Array.new(9).collect{ 3.0*(rand()-0.5) }
      c = -1
      p_as_str = ""
      p.each_slice(3){ |s| p_as_str += "#{c += 1} #{s.join(" ")} " }
      
      response_to("set position #{p_as_str}").should match p.join(" ")
      response_to("get position *").should match p.join(" ")
      (0..3).each do |i|
        response_to("get position #{i}").should match p[3*i..(3*i+2)].join(" ")
      end
    end

    it "should set positions en masse" do
      # out of order - response is in the order set
      response_to("set position 1 1.0 2.0 3.0 0 -1.0 -2.0 -3.0").
        should match "1.0 2.0 3.0 -1.0 -2.0 -3.0"
      # should return in order, 2 and 3 should still be 0
      response_to("get position *").should match("-1.0 -2.0 -3.0 1.0 2.0 3.0 " + ("0.0 "*6).strip)
      response_to("get position 2").should match "0.0 0.0 0.0"
    end
    
  end

  describe "controls" do

    it "should get the controls" do
      response_to("get control 0").should match "waypoint 0.0 0.0 0.0"
    end

    it "should set the controls" do
      response_to("set control waypoint 0 1.0 2.0 3.0").should match "waypoint 1.0 2.0 3.0"
    end

    it "should set the control mode" do
      response_to("set control kinematics").should match "kinematics"
      response_to("get control 0").should match "kinematics 0.0 0.0 0.0"
    end

    it "should set the controls - waypoints (random data)" do
      (0..3).each do |i|
        p = Array.new(3).collect{ 3.0*(rand()-0.5) }
        p_as_str = p.join(" ")
        response_to("set control waypoint #{i} #{p_as_str}").should match "waypoint #{p_as_str}"
      end
    end

    it "should set the controls - kinematics (random data)" do
      (0..3).each do |i|
        p = Array.new(3).collect{ 3.0*(rand()-0.5) }
        p_as_str = p.join(" ")
        response_to("set control kinematics #{i} #{p_as_str}").
          should match "kinematics #{p_as_str}"
      end
    end

    it "should set positions en masse - waypoint (random data)" do
      p = Array.new(9).collect{ 3.0*(rand()-0.5) }
      c = -1
      p_as_str = ""
      p.each_slice(3){ |s| p_as_str += "#{c += 1} #{s.join(" ")} " }
      
      response_to("set control waypoint #{p_as_str}").should match p.join(" ")
      response_to("get control *").should match "waypoint " + p.join(" ")
      (0..3).each do |i|
        response_to("get control #{i}").should match "waypoint " + p[3*i..(3*i+2)].join(" ")
      end
    end

    it "should set positions en masse - kinematics (random data)" do
      p = Array.new(9).collect{ 3.0*(rand()-0.5) }
      c = -1
      p_as_str = ""
      p.each_slice(3){ |s| p_as_str += "#{c += 1} #{s.join(" ")} " }
      
      response_to("set control kinematics #{p_as_str}").should match p.join(" ")
      response_to("get control *").should match "kinematics " + p.join(" ")
      (0..3).each do |i|
        response_to("get control #{i}").should match "kinematics " + p[3*i..(3*i+2)].join(" ")
      end
    end    
    
  end

  after(:each) do
    @sock.puts "S"
    @sock.gets
    @sock.close
    @server.join
  end

  # just a helper function for the tests
  def response_to send_string
    @sock.puts send_string
    return @sock.gets
  end
  
end

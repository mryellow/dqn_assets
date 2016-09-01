ros = require 'ros'
local classic = require 'classic'

local Kulbabu, super = classic.class('Kulbabu', Env)

-- Constructor
function Kulbabu:_init(opts)
  opts = opts or {}

  self.width = opts.width or 8
  self.height = opts.height or 1
  -- TODO: Second channel for goal direction/distance.
  self.channels = 1
  self.screen = torch.Tensor(self.channels, self.height, self.width):zero()

  self.ns = "kulbabu"
  if __threadid then
    self.ns = self.ns .. __threadid
  else
    self.ns = self.ns .. "0"
  end
  self.range_topics = {
    "range_1l",
    "range_2l",
    "range_3l",
    "range_4l",
    "range_1r",
    "range_2r",
    "range_3r",
    "range_4r"
  }
  self.range_msg = "sensor_msgs/Range"

  self.cmd_vel_topic = "/" .. self.ns .. "/diff_drive_controller/cmd_vel"
  self.cmd_vel_msg = ros.MsgSpec('geometry_msgs/Twist')

  self.subs = {}
  self.pubs = {}

  ros.init(self.ns .. "_dqn")

  spinner = ros.AsyncSpinner()
  spinner:start()

  self.nh = ros.NodeHandle()
end

function Kulbabu:getStateSpec()
  return {'real', {self.channels, self.height, self.width}, {0, 1}}
end

-- forward, left-forward, right-forward, left, right
function Kulbabu:getActionSpec()
  return {'int', 1, {0, 4}}
end

function Kulbabu:getDisplaySpec()
  return {'real', {self.channels, self.height, self.width}, {0, 1}}
end

-- Min and max reward
function Kulbabu:getRewardSpec()
  return 0, 1
end

-- Starts new game
function Kulbabu:start()
  -- Reset screen
  self.screen:zero()

  -- Subscribe to ROS topics
  self:createSubs()

  -- Setup ROS publishers
  self:createPubs()

  -- Return observation
  return self.screen
end

-- Steps in a game
function Kulbabu:step(action)
  -- Reward is 0 by default
  local reward = 0

  -- Spin ROS and get messages
  if ros.ok() then
    ros.spinOnce()
  end

  -- Publish twists for actions
  self:pubAction(action)

  -- TODO: Calculate reward based on reaching goal
  -- Will need subscription to states and trig for relative position.

  -- TODO: Check terminal condition
  local terminal = false

  return reward, self.screen, terminal
end

-- Returns (RGB) display of screen
function Kulbabu:getDisplay()
  --return torch.repeatTensor(self.screen, 3, 1, 1)
  return self.screen
end

function Kulbabu:createSubs()
  self:destroySubs()

  -- Subscribe to range sensor topics and update state
  for i, topic in ipairs(self.range_topics) do
    log.info("Subscribe: /" .. self.ns .. "/" .. topic)
    subscriber = self.nh:subscribe("/" .. self.ns .. "/" .. topic, self.range_msg, 100)
    subscriber:registerCallback(function(msg, header)
      --log.info(msg.range / msg.max_range)
      self.screen[{{1}, {1}, {i}}] = msg.range / msg.max_range
    end)
    table.insert(self.subs, subscriber)
  end

  -- TODO: Subscribe to states for goal relative position
end

-- Destroy any existing subscriptions
function Kulbabu:destroySubs()
  for i, subscriber in ipairs(self.subs) do
    subscriber:shutdown()
  end
  self.subs = {}
end

function Kulbabu:createPubs()
  self:destroyPubs()

  publisher = self.nh:advertise(self.cmd_vel_topic, self.cmd_vel_msg, 100, false, connect_cb, disconnect_cb)
  table.insert(self.pubs, publisher)
  ros.spinOnce()
end

-- Destroy any existing publishers
function Kulbabu:destroyPubs()
  for i, publisher in ipairs(self.pubs) do
    publisher:shutdown()
  end
  self.pubs = {}
end

function connect_cb(name, topic)
  print("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  print("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

function Kulbabu:pubAction(action)
  msg = ros.Message(self.cmd_vel_msg)
  publisher = self.pubs[1]
  if publisher:getNumSubscribers() == 0 then
    --log.info('waiting for subscriber')
  else
    -- Forward
    if action == 0 then
      msg.linear.x = 1.00
      msg.angular.z = 0.00
    -- Forward-left
    elseif action == 1 then
      msg.linear.x = 1.00
      msg.angular.z = -0.80
    -- Forward-right
    elseif action == 2 then
      msg.linear.x = 1.00
      msg.angular.z = 0.80
    -- Left
    elseif action == 3 then
      msg.linear.x = 0.00
      msg.angular.z = -1.00
    -- Right
    elseif action == 4 then
      msg.linear.x = 0.00
      msg.angular.z = 1.00
    end
    --log.info(msg)
    publisher:publish(msg)
  end
end

-- TODO: Capture sigint destroy pub/subs and `ros.shutdown()``

return Kulbabu

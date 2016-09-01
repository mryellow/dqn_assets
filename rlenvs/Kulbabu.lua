ros = require 'ros'
local classic = require 'classic'

local Kulbabu, super = classic.class('Kulbabu', Env)

-- Constructor
function Kulbabu:_init(opts)
  opts = opts or {}

  self.width = opts.width or 8
  self.height = opts.height or 1
  -- Second channel for goal direction/distance.
  self.channels = 2
  self.screen = torch.Tensor(self.channels, self.height, self.width):zero()

  -- Max-range for goal sensor
  self.goal_max = 10

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

  self.robot_pose = {
    position = {
      x = 0,
      y = 0,
      z = 0
    },
    orientation = {
      x = 0,
      y = 0,
      z = 0,
      w = 0
    }
  }
  self.goal_pose = {
    position = {
      x = 0,
      y = 0,
      z = 0
    },
    orientation = {
      x = 0,
      y = 0,
      z = 0,
      w = 0
    }
  }

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

  -- Get/update relative location
  local rad, dis = self:goalLocation()
  self:refreshGoal(rad, dis)

  -- Calculate reward based on reaching goal
  reward = math.min(1,1 - (dis / self.goal_max))
  --log.info("Reward: " .. reward)

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

  local subscriber
  -- Subscribe to range sensor topics and update state
  for i, topic in ipairs(self.range_topics) do
    log.info("Subscribe: /" .. self.ns .. "/" .. topic)
    subscriber = self.nh:subscribe("/" .. self.ns .. "/" .. topic, self.range_msg, 100)
    subscriber:registerCallback(function(msg, header)
      --log.info(msg.range / msg.max_range)
      self.screen[{{1}, {1}, {i}}] = math.min(1,1 - (msg.range / msg.max_range))
    end)
    table.insert(self.subs, subscriber)
  end

  -- Subscribe to states for goal relative position
  subscriber = self.nh:subscribe("/gazebo/model_states", "gazebo_msgs/ModelStates", 100)
  subscriber:registerCallback(function(msg, header)
    local robot_key = get_key_for_value(msg.name, self.ns)
    --log.info("robot: " .. msg.pose[robot_key].position.x .. "/" .. msg.pose[robot_key].position.y)
    self.robot_pose.position = msg.pose[robot_key].position
    self.robot_pose.orientation = msg.pose[robot_key].orientation
    local goal_key = get_key_for_value(msg.name, self.ns .. "/goal")
    --log.info("goal: " .. msg.pose[goal_key].position.x .. "/" .. msg.pose[goal_key].position.y)
    self.goal_pose.position = msg.pose[goal_key].position
    self.goal_pose.orientation = msg.pose[goal_key].orientation
  end)
  table.insert(self.subs, subscriber)
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

  local publisher = self.nh:advertise(self.cmd_vel_topic, self.cmd_vel_msg, 100, false, connect_cb, disconnect_cb)
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
  log.debug("subscriber connected: " .. name .. " (topic: '" .. topic .. "')")
end

function disconnect_cb(name, topic)
  log.debug("subscriber diconnected: " .. name .. " (topic: '" .. topic .. "')")
end

function Kulbabu:pubAction(action)
  local msg = ros.Message(self.cmd_vel_msg)
  local publisher = self.pubs[1]
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

function Kulbabu:goalLocation()
  --`tan(rad) = Opposite / Adjacent = (y2-y1)/(x2-x1)`
  local rad = math.atan2(
    self.goal_pose.position.y - self.robot_pose.position.y,
    self.goal_pose.position.x - self.robot_pose.position.x
  )

  -- `Hypotenuse = (y2-y1)/sin(rad)`
  local dis = math.abs(
    (self.goal_pose.position.y - self.robot_pose.position.y)/math.sin(rad)
  )

  -- Extract angles from quaternion.
  -- http://stackoverflow.com/a/18115837/2438830
  local q = self.robot_pose.orientation
  local robot_yaw = math.atan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

-- Minus robot pose from goal direction.
  rad = rad - robot_yaw
  if rad > math.pi then
    rad = rad - (2 * math.pi)
  elseif rad < -math.pi then
    rad = rad + (2 * math.pi)
  end

  return rad, dis
end

function Kulbabu:refreshGoal(rad, dis)
  if self.channels > 1 and dis > 0 then
    local fov = (2*math.pi)/self.width
    for x=1,self.width do
      local seg_begin  = -math.pi+(fov*(x-1))
      local seg_finish = seg_begin + fov
      if seg_begin < rad and seg_finish >= rad then
        self.screen[{{2}, {1}, {x}}] = math.min(1,1 - (dis / self.goal_max))
      else
        self.screen[{{2}, {1}, {x}}] = 0
      end
    end
  end
end

-- TODO: Capture sigint destroy pub/subs and `ros.shutdown()``

function get_key_for_value(t, value)
  for k,v in pairs(t) do
    if v==value then
      return k
    end
  end
  return nil
end

return Kulbabu
